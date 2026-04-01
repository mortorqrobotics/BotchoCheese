// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchoCheese;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.BotchoCheese.Commands.RotateToTag;
import frc.BotchoCheese.Commands.StrafeToTag;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Constants.TunerConstants;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Subsystems.Feeder;
import frc.BotchoCheese.Subsystems.Indexer;
import frc.BotchoCheese.Subsystems.Intake;
import frc.BotchoCheese.Subsystems.Pivot;
import frc.BotchoCheese.Subsystems.Shooter;
import frc.BotchoCheese.Utils.DebugLog;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class RobotContainer {
    // Auto chooser/dashboard
    private static final String NO_AUTO_SELECTED = "Select Auto";
    private static final String AUTO_CHOOSER_KEY = "Auto Mode";
    private static final String PATHPLANNER_AUTO_FOLDER = "pathplanner/autos";
    private static final String PATHPLANNER_PATHS_FOLDER = "pathplanner/paths";

    // Drive tuning
    private static final double DRIVE_DEADBAND = 0.1;
    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Controllers
    private static final CommandXboxController JOYSTICK1_CONTROLLER = new CommandXboxController(0);
    private static final CommandXboxController JOYSTICK2_CONTROLLER = new CommandXboxController(1);

    // Drivetrain + sensors
    public static final CommandSwerveDrivetrain drivetrain = createDrivetrain();
    public static Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_ID);

    // Subsystems
    public final Shooter shooter = new Shooter();
    public final Feeder feeder = new Feeder();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Pivot pivot = new Pivot();

    // Blue-side reference poses for teleop pathfind shot setpoints.
    // These are loaded from linked waypoints and flipped automatically for Red.
    private Pose2d leftHubShootingPoseBlue;
    private Pose2d middleHubShootingPoseBlue;
    private Pose2d rightHubShootingPoseBlue;

    // Drive requests
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * DRIVE_DEADBAND)
        .withRotationalDeadband(MaxAngularRate * DRIVE_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Auto selection
    private final SendableChooser<String> autoChooser;
    private Command activeDriverPathfindCommand;

    public RobotContainer() {
        registerNamedCommands();
        loadShotSetpointsFromLinkedWaypoints();

        autoChooser = new SendableChooser<>();
        configureAutoChooser();
        configureDashboard();

        configureBindings();
    }

    private void registerNamedCommands() {
        final double pivotDownSeconds = 1.0; // tune this "X seconds" value

        NamedCommands.registerCommand(
            "Shoot",
            Commands.sequence(
                shooter.shootRps(75.0).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(75.0),
                    intake.runIntake(0.75),
                    indexer.runIndexer(0.85),
                    feeder.runFeeder(0.75)
                )
            ).withTimeout(10.0)
        );

        NamedCommands.registerCommand("PivotDown", pivot.pivotDown().withTimeout(pivotDownSeconds));
        NamedCommands.registerCommand(
            "Intake",
            Commands.parallel(
                intake.runIntake(0.75),
                indexer.runIndexer(-0.5),
                feeder.runFeeder(-0.75),
                shooter.shootRps(0.0, -25.0)
            )
        );
    }

    private void configureDashboard() {
        SmartDashboard.putData(AUTO_CHOOSER_KEY, autoChooser);
        SmartDashboard.putData("Zero Pivot Encoder", new InstantCommand(pivot::zeroPivotEncoder, pivot));
    }

    private void configureAutoChooser() {
        List<String> autoNames = getAutoNamesFromDeploy();

        autoChooser.setDefaultOption(NO_AUTO_SELECTED, NO_AUTO_SELECTED);

        if (autoNames.isEmpty()) {
            DebugLog.warnThrottled(
                "autos.none_found",
                "No PathPlanner autos found in deploy/pathplanner/autos",
                10.0
            );
            return;
        }

        for (String autoName : autoNames) {
            autoChooser.addOption(autoName, autoName);
        }
    }

    private List<String> getAutoNamesFromDeploy() {
        Path autoFolder = Filesystem.getDeployDirectory().toPath().resolve(PATHPLANNER_AUTO_FOLDER);
        if (!Files.isDirectory(autoFolder)) {
            return List.of();
        }

        try (var files = Files.list(autoFolder)) {
            List<String> autoNames = files
                .filter(path -> path.toString().endsWith(".auto"))
                .map(path -> path.getFileName().toString().replaceFirst("\\.auto$", ""))
                .sorted(Comparator.naturalOrder())
                .collect(Collectors.toList());

            List<String> nonRedAutoNames = autoNames.stream()
                .filter(name -> !name.toLowerCase().startsWith("red"))
                .collect(Collectors.toList());

            if (nonRedAutoNames.size() != autoNames.size()) {
                DebugLog.warnThrottled(
                    "autos.red_filtered",
                    "Ignoring Red-prefixed autos in chooser; using blue-side autos with alliance flip instead.",
                    10.0
                );
            }

            return nonRedAutoNames;
        } catch (IOException e) {
            DebugLog.error("Failed to read PathPlanner autos: " + e.getMessage(), e.getStackTrace());
            return List.of();
        }
    }

    private void loadShotSetpointsFromLinkedWaypoints() {
        LinkedWaypointPose left = findLinkedWaypointPose(Set.of("Left shoot"));
        if (left != null) {
            leftHubShootingPoseBlue = left.pose();
        }

        LinkedWaypointPose middle = findLinkedWaypointPose(Set.of("Middle shoot"));
        if (middle != null) {
            middleHubShootingPoseBlue = middle.pose();
        }

        // Keep compatibility with existing path files that currently use "Right shooting".
        LinkedWaypointPose right = findLinkedWaypointPose(Set.of("Right shoot", "Right shooting"));
        if (right != null) {
            rightHubShootingPoseBlue = right.pose();
        }
    }

    private LinkedWaypointPose findLinkedWaypointPose(Set<String> linkedNames) {
        Path pathsFolder = Filesystem.getDeployDirectory().toPath().resolve(PATHPLANNER_PATHS_FOLDER);
        if (!Files.isDirectory(pathsFolder)) {
            DebugLog.warnThrottled(
                "paths.folder_missing",
                "PathPlanner paths folder not found for linked setpoint lookup: " + pathsFolder,
                10.0
            );
            return null;
        }

        try (var files = Files.list(pathsFolder)) {
            List<Path> pathFiles = files
                .filter(path -> path.toString().endsWith(".path"))
                .sorted(Comparator.naturalOrder())
                .collect(Collectors.toList());

            JSONParser parser = new JSONParser();
            for (Path pathFile : pathFiles) {
                String content = Files.readString(pathFile);
                JSONObject root = (JSONObject) parser.parse(content);
                JSONArray waypoints = (JSONArray) root.get("waypoints");
                if (waypoints == null) {
                    continue;
                }

                double rotationDeg = 180.0;
                JSONObject goalEndState = (JSONObject) root.get("goalEndState");
                if (goalEndState != null && goalEndState.get("rotation") instanceof Number) {
                    rotationDeg = ((Number) goalEndState.get("rotation")).doubleValue();
                }

                for (Object waypointObj : waypoints) {
                    if (!(waypointObj instanceof JSONObject waypoint)) {
                        continue;
                    }

                    Object linkedNameObj = waypoint.get("linkedName");
                    if (!(linkedNameObj instanceof String linkedName) || !linkedNames.contains(linkedName)) {
                        continue;
                    }

                    JSONObject anchor = (JSONObject) waypoint.get("anchor");
                    if (anchor == null || !(anchor.get("x") instanceof Number) || !(anchor.get("y") instanceof Number)) {
                        continue;
                    }

                    double x = ((Number) anchor.get("x")).doubleValue();
                    double y = ((Number) anchor.get("y")).doubleValue();
                    Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDeg));
                    return new LinkedWaypointPose(pose, pathFile.getFileName().toString());
                }
            }
        } catch (Exception ex) {
            DebugLog.error("Failed to read linked shot setpoints from PathPlanner paths", ex.getStackTrace());
        }

        return null;
    }

    private static double applyDriveDeadband(double value) {
        return MathUtil.applyDeadband(value, DRIVE_DEADBAND);
    }

    private void configureBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        final PathConstraints teleopPathfindConstraints = new PathConstraints(2.5, 2.0, 4.0, 6.0);

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftY()) * MaxSpeed)
                    .withVelocityY(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftX()) * MaxSpeed)
                    .withRotationalRate(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getRightX()) * MaxAngularRate)
            )
        );

        JOYSTICK1_CONTROLLER.leftBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        JOYSTICK1_CONTROLLER.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        JOYSTICK1_CONTROLLER.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        JOYSTICK1_CONTROLLER.povLeft().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        JOYSTICK1_CONTROLLER.povRight().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );

        JOYSTICK1_CONTROLLER.start().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

        // Teleop pathfind shot setpoints
        JOYSTICK1_CONTROLLER.x().onTrue(new InstantCommand(
            () -> scheduleShotPathIfConfigured("Left", leftHubShootingPoseBlue, teleopPathfindConstraints)
        ));
        JOYSTICK1_CONTROLLER.x().onFalse(new InstantCommand(this::cancelActiveDriverPathfind));

        JOYSTICK1_CONTROLLER.y().onTrue(new InstantCommand(
            () -> scheduleShotPathIfConfigured("Middle", middleHubShootingPoseBlue, teleopPathfindConstraints)
        ));
        JOYSTICK1_CONTROLLER.y().onFalse(new InstantCommand(this::cancelActiveDriverPathfind));

        JOYSTICK1_CONTROLLER.b().onTrue(new InstantCommand(
            () -> scheduleShotPathIfConfigured("Right", rightHubShootingPoseBlue, teleopPathfindConstraints)
        ));
        JOYSTICK1_CONTROLLER.b().onFalse(new InstantCommand(this::cancelActiveDriverPathfind));

        JOYSTICK1_CONTROLLER.leftTrigger().whileTrue(new StrafeToTag(drivetrain));
        JOYSTICK1_CONTROLLER.rightTrigger().whileTrue(new RotateToTag(drivetrain, 0));
    }

    private void configureOperatorBindings() {
        final double pivotUpForShotRotations = 6.0;
        final double pivotToggleTimeoutSeconds = 0.6;

        // Pivot controls
        JOYSTICK2_CONTROLLER.povUp().whileTrue(pivot.pivotUp());
        JOYSTICK2_CONTROLLER.povDown().whileTrue(pivot.pivotDown());
        JOYSTICK2_CONTROLLER.povLeft().whileTrue(pivot.pivotToBottomAndHome());
        JOYSTICK2_CONTROLLER.povRight().whileTrue(
            Commands.sequence(
                pivot.pivotUpToRotations(pivotUpForShotRotations),
                Commands.repeatingSequence(
                    pivot.pivotDown().withTimeout(pivotToggleTimeoutSeconds),
                    pivot.pivotUp().withTimeout(pivotToggleTimeoutSeconds)
                )
            )
        );

        // Intake balls / anti-jam
        JOYSTICK2_CONTROLLER.leftTrigger().toggleOnTrue(
            Commands.parallel(
                intake.runIntake(0.75),
                indexer.runIndexer(-0.5),
                feeder.runFeeder(-0.75),
                shooter.shootRps(0.0, -25.0)
            )
        );

        // Reverse all conveyors
        JOYSTICK2_CONTROLLER.b().whileTrue(
            Commands.parallel(
                intake.runIntake(-0.75),
                indexer.runIndexer(-0.75),
                feeder.runFeeder(-0.5)
            )
        );

        // Big shot
        JOYSTICK2_CONTROLLER.rightTrigger().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(120.0).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(120.0),
                    intake.runIntake(0.75),
                    indexer.runIndexer(0.75),
                    feeder.runFeeder(0.75)
                )
            )
        );

        // Regular shot
        JOYSTICK2_CONTROLLER.rightBumper().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(75.0).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(75.0),
                    intake.runIntake(0.75),
                    indexer.runIndexer(0.85),
                    feeder.runFeeder(0.75)
                )
            )
        );

        // Lob shot (back, front)
        JOYSTICK2_CONTROLLER.y().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(120.0, 6.0).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(120.0, 6.0),
                    intake.runIntake(0.75),
                    indexer.runIndexer(0.85),
                    feeder.runFeeder(0.75)
                )
            )
        );

        // Line-drive shot (back, front)
        JOYSTICK2_CONTROLLER.a().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(6.0, 120.0).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(6.0, 120.0),
                    intake.runIntake(0.75),
                    indexer.runIndexer(0.85),
                    feeder.runFeeder(0.75)
                )
            )
        );
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = autoChooser.getSelected();
        if (selectedAutoName == null || selectedAutoName.equals(NO_AUTO_SELECTED)) {
            DebugLog.warnThrottled(
                "auto.none_selected",
                "No autonomous selected; running no-op command.",
                5.0
            );
            return Commands.none();
        }
        DebugLog.info("Auto selected: " + selectedAutoName);
        return AutoBuilder.buildAuto(selectedAutoName);
    }

    public void seedPoseFromSelectedAuto() {
        String selectedAutoName = autoChooser.getSelected();
        if (selectedAutoName == null || selectedAutoName.equals(NO_AUTO_SELECTED)) {
            DebugLog.warnThrottled(
                "pose_seed.no_auto",
                "No auto selected for pose seeding.",
                5.0
            );
            return;
        }

        try {
            PathPlannerAuto auto = new PathPlannerAuto(selectedAutoName);
            Pose2d bluePose = auto.getStartingPose();
            if (bluePose == null) {
                DebugLog.warnThrottled(
                    "pose_seed.no_start_pose",
                    "Selected auto has no path-based starting pose: " + selectedAutoName,
                    5.0
                );
                return;
            }

            Pose2d alliancePose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
            drivetrain.resetPose(alliancePose);
            DebugLog.info("Seeded pose from auto: " + selectedAutoName);
        } catch (Exception ex) {
            DebugLog.error(
                "Failed to seed pose from selected auto: " + selectedAutoName,
                ex.getStackTrace()
            );
        }
    }

    private record LinkedWaypointPose(Pose2d pose, String sourcePathFile) {}

    private void scheduleShotPathIfConfigured(String name, Pose2d bluePose, PathConstraints constraints) {
        if (bluePose == null) {
            DebugLog.warnThrottled(
                "shot_setpoint_missing_" + name.toLowerCase(),
                "Shot setpoint not loaded for " + name + ". Button press ignored.",
                2.0
            );
            return;
        }

        cancelActiveDriverPathfind();
        activeDriverPathfindCommand = AutoBuilder.pathfindToPoseFlipped(bluePose, constraints);
        activeDriverPathfindCommand.schedule();
    }

    private void cancelActiveDriverPathfind() {
        if (activeDriverPathfindCommand != null && activeDriverPathfindCommand.isScheduled()) {
            activeDriverPathfindCommand.cancel();
        }
    }

    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants, 0,
            VecBuilder.fill(RobotMap.kPositionStdDevX, RobotMap.kPositionStdDevY, Units.degreesToRadians(RobotMap.kPositionStdDevTheta)),
            VecBuilder.fill(RobotMap.kVisionStdDevX, RobotMap.kVisionStdDevY, Units.degreesToRadians(RobotMap.kVisionStdDevTheta)),
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
        );
    }
}
