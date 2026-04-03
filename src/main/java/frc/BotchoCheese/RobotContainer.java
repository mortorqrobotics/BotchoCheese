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
import java.util.stream.Collectors;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Constants.TunerConstants;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Subsystems.Feeder;
import frc.BotchoCheese.Subsystems.Indexer;
import frc.BotchoCheese.Subsystems.Intake;
import frc.BotchoCheese.Subsystems.Pivot;
import frc.BotchoCheese.Subsystems.Shooter;
import frc.BotchoCheese.Utils.DebugLog;

public class RobotContainer {
    // Auto chooser/dashboard
    private static final String NO_AUTO_SELECTED = "Select Auto";
    private static final String AUTO_CHOOSER_KEY = "Auto Mode";
    private static final String PATHPLANNER_AUTO_FOLDER = "pathplanner/autos";
    private static final String X_SHOT_BACK_RPS_KEY = "Shots/X Back RPS";
    private static final String X_SHOT_FRONT_RPS_KEY = "Shots/X Front RPS";
    private static final String AUTO_SELECTED_NAME_KEY = "Auto/SelectedName";
    private static final String AUTO_SELECTED_VALID_KEY = "Auto/SelectedValid";
    private static final String AUTO_STATUS_KEY = "Auto/Status";
    private static final String AUTO_START_POSE_SEEDED_KEY = "Auto/StartPoseSeeded";

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

    public RobotContainer() {
        registerNamedCommands();

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
        SmartDashboard.putNumber(X_SHOT_BACK_RPS_KEY, 75.0);
        SmartDashboard.putNumber(X_SHOT_FRONT_RPS_KEY, 75.0);
        SmartDashboard.putString(AUTO_SELECTED_NAME_KEY, NO_AUTO_SELECTED);
        SmartDashboard.putBoolean(AUTO_SELECTED_VALID_KEY, false);
        SmartDashboard.putString(AUTO_STATUS_KEY, "NO AUTO SELECTED");
        SmartDashboard.putBoolean(AUTO_START_POSE_SEEDED_KEY, false);
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
            return autoNames;
        } catch (IOException e) {
            DebugLog.error("Failed to read PathPlanner autos: " + e.getMessage(), e.getStackTrace());
            return List.of();
        }
    }

    private static double applyDriveDeadband(double value) {
        return MathUtil.applyDeadband(value, DRIVE_DEADBAND);
    }

    private void configureBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftY()) * MaxSpeed)
                    .withVelocityY(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftX()) * MaxSpeed)
                    .withRotationalRate(applyDriveDeadband(JOYSTICK1_CONTROLLER.getRightX()) * MaxAngularRate)
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

        JOYSTICK1_CONTROLLER.start().onTrue(new InstantCommand(()->drivetrain.seedFieldCentric()));
    }

    private void configureOperatorBindings() {

        // Pivot controls
        JOYSTICK2_CONTROLLER.povUp().whileTrue(pivot.pivotUp());
        JOYSTICK2_CONTROLLER.povDown().whileTrue(pivot.pivotDown());

        // Intake balls / anti-jam
        JOYSTICK2_CONTROLLER.leftTrigger().whileTrue(
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
                    intake.runIntake(0.9),
                    indexer.runIndexer(0.75),
                    feeder.runFeeder(0.85)
                )
            )
        );

        // Regular shot
        JOYSTICK2_CONTROLLER.rightBumper().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(90.0).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(90.0),
                    intake.runIntake(0.9),
                    indexer.runIndexer(0.85),
                    feeder.runFeeder(0.85)
                )
            )
        );

        // SmartDashboard-programmed X shot
        JOYSTICK2_CONTROLLER.x().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(getXShotBackRps(), getXShotFrontRps()).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(getXShotBackRps(), getXShotFrontRps()),
                    intake.runIntake(0.9),
                    indexer.runIndexer(0.85),
                    feeder.runFeeder(0.85)
                )
            )
        );

        // Lob shot (back, front)
        JOYSTICK2_CONTROLLER.y().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(120.0, 20).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(120.0, 20),
                    intake.runIntake(0.9),
                    indexer.runIndexer(0.85),
                    feeder.runFeeder(0.85)
                )
            )
        );

        // Line-drive shot (back, front)
        JOYSTICK2_CONTROLLER.a().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(20, 120.0).withTimeout(0.5),
                Commands.parallel(
                    shooter.shootRps(20, 120.0),
                    intake.runIntake(0.9),
                    indexer.runIndexer(0.85),
                    feeder.runFeeder(0.85)
                )
            )
        );
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = autoChooser.getSelected();
        if (!isAutoSelected(selectedAutoName)) {
            setAutoStatus("NO AUTO SELECTED");
            DebugLog.warnThrottled(
                "auto.none_selected",
                "No autonomous selected; running no-op command.",
                5.0
            );
            return Commands.none();
        }
        try {
            Command autoCommand = AutoBuilder.buildAuto(selectedAutoName);
            setAutoStatus("AUTO READY: " + selectedAutoName);
            DebugLog.info("Auto selected: " + selectedAutoName);
            return autoCommand;
        } catch (Exception ex) {
            setAutoStatus("AUTO BUILD FAILED: " + selectedAutoName);
            DebugLog.error(
                "Failed to build autonomous command: " + selectedAutoName,
                ex.getStackTrace()
            );
            return Commands.none();
        }
    }

    public void updateAutoSelectionDashboard() {
        String selectedAutoName = autoChooser.getSelected();
        boolean autoSelected = isAutoSelected(selectedAutoName);

        SmartDashboard.putString(
            AUTO_SELECTED_NAME_KEY,
            autoSelected ? selectedAutoName : NO_AUTO_SELECTED
        );
        SmartDashboard.putBoolean(AUTO_SELECTED_VALID_KEY, autoSelected);
        if (!autoSelected) {
            setAutoStatus("NO AUTO SELECTED");
            SmartDashboard.putBoolean(AUTO_START_POSE_SEEDED_KEY, false);
        }
    }

    public boolean seedPoseFromSelectedAuto() {
        String selectedAutoName = autoChooser.getSelected();
        if (!isAutoSelected(selectedAutoName)) {
            SmartDashboard.putBoolean(AUTO_START_POSE_SEEDED_KEY, false);
            setAutoStatus("NO AUTO SELECTED");
            DebugLog.warnThrottled(
                "pose_seed.no_auto",
                "No auto selected for pose seeding.",
                5.0
            );
            return false;
        }

        try {
            PathPlannerAuto auto = new PathPlannerAuto(selectedAutoName);
            Pose2d bluePose = auto.getStartingPose();
            if (bluePose == null) {
                SmartDashboard.putBoolean(AUTO_START_POSE_SEEDED_KEY, false);
                setAutoStatus("AUTO HAS NO START POSE: " + selectedAutoName);
                DebugLog.warnThrottled(
                    "pose_seed.no_start_pose",
                    "Selected auto has no path-based starting pose: " + selectedAutoName,
                    5.0
                );
                return false;
            }

            Pose2d alliancePose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
            drivetrain.resetPose(alliancePose);
            SmartDashboard.putBoolean(AUTO_START_POSE_SEEDED_KEY, true);
            setAutoStatus("START POSE SEEDED: " + selectedAutoName);
            DebugLog.info("Seeded pose from auto: " + selectedAutoName);
            return true;
        } catch (Exception ex) {
            SmartDashboard.putBoolean(AUTO_START_POSE_SEEDED_KEY, false);
            setAutoStatus("POSE SEED FAILED: " + selectedAutoName);
            DebugLog.error(
                "Failed to seed pose from selected auto: " + selectedAutoName,
                ex.getStackTrace()
            );
            return false;
        }
    }

    private boolean isAutoSelected(String selectedAutoName) {
        return selectedAutoName != null && !selectedAutoName.equals(NO_AUTO_SELECTED);
    }

    private void setAutoStatus(String status) {
        SmartDashboard.putString(AUTO_STATUS_KEY, status);
    }

    private double getXShotBackRps() {
        return SmartDashboard.getNumber(X_SHOT_BACK_RPS_KEY, 75.0);
    }

    private double getXShotFrontRps() {
        return SmartDashboard.getNumber(X_SHOT_FRONT_RPS_KEY, 75.0);
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
