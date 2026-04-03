// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchoCheese;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
    private static final String AUTO_CHOOSER_KEY = "Auto Chooser";
    private static final String LEGACY_AUTO_CHOOSER_KEY = "Auto Mode";
    private static final String X_SHOT_BACK_RPS_KEY = "Shots/X Back RPS";
    private static final String X_SHOT_FRONT_RPS_KEY = "Shots/X Front RPS";
    private static final String AUTO_SELECTED_NAME_KEY = "Auto/SelectedName";
    private static final String AUTO_SELECTED_VALID_KEY = "Auto/SelectedValid";
    private static final String AUTO_STATUS_KEY = "Auto/Status";
    private static final String AUTO_START_POSE_SEEDED_KEY = "Auto/StartPoseSeeded";

    // Driver input deadband and drivetrain speed caps used by the default drive command.
    private static final double DRIVE_DEADBAND = 0.1;
    private static final double DRIVER_SLOW_ROTATE_SCALE = 0.2;
    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private static final double DRIVER_SLOW_ROTATE_RATE = MaxAngularRate * DRIVER_SLOW_ROTATE_SCALE;
    // Shared duty cycles for all shoot flows (buttons + named commands).
    private static final double SHOOT_INTAKE_DUTY = 0.6;
    private static final double SHOOT_INDEXER_DUTY = 0.5;
    private static final double SHOOT_FEEDER_DUTY = 0.7;
    // Brownout mitigation for intake-assist/reverse flows: keep intake stronger than feeder/indexer.
    private static final double INTAKE_ASSIST_INTAKE_DUTY = 0.75;
    private static final double INTAKE_ASSIST_INDEXER_DUTY = -0.5;
    private static final double INTAKE_ASSIST_FEEDER_DUTY = -0.1;
    private static final double REVERSE_INTAKE_DUTY = -0.6;
    private static final double REVERSE_INDEXER_DUTY = -0.5;
    private static final double REVERSE_FEEDER_DUTY = -0.1;
    private static final double SHOOTER_SPINUP_TIMEOUT_SECONDS = 0.8;

    // USB controller ports: driver on 0, operator on 1.
    private static final CommandXboxController JOYSTICK1_CONTROLLER = new CommandXboxController(0);
    private static final CommandXboxController JOYSTICK2_CONTROLLER = new CommandXboxController(1);

    // Shared drivetrain instance plus a standalone pigeon handle using the mapped device ID.
    public static final CommandSwerveDrivetrain drivetrain = createDrivetrain();
    public static Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_ID);

    // Mechanism subsystems used by button bindings and autos.
    public final Shooter shooter = new Shooter();
    public final Feeder feeder = new Feeder();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Pivot pivot = new Pivot();

    // Default driver request is field-centric open-loop drive.
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * DRIVE_DEADBAND)
        .withRotationalDeadband(MaxAngularRate * DRIVE_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Brake request locks the swerve modules in place while the button is held.
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // Robot-centric request used for the D-pad cardinal-direction nudges.
    private final SwerveRequest.RobotCentric robotCentricNudge = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Separate request used for slow in-place rotation on driver triggers.
    private final SwerveRequest.RobotCentric robotCentricRotate = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Auto selection
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        configureAutoChooser();
        configureDashboard();

        configureBindings();
    }

    private void registerNamedCommands() {
        final double pivotDownSeconds = 0.8; // tune this "X seconds" value
        final double autoIntakeSecondsAfterPivotDown = 5.0;

        // These names must match the event markers referenced by PathPlanner autos.
        NamedCommands.registerCommand(
            "Shoot",
            Commands.sequence(
                shooter.shootRps(90.0).withTimeout(SHOOTER_SPINUP_TIMEOUT_SECONDS),
                Commands.parallel(
                    shooter.shootRps(90.0),
                    intake.runIntake(SHOOT_INTAKE_DUTY),
                    indexer.runIndexer(SHOOT_INDEXER_DUTY),
                    feeder.runFeeder(SHOOT_FEEDER_DUTY)
                )
            ).withTimeout(3.0)
        );

        Command pivotDownAndRun = Commands.sequence(
            pivot.pivotDown().withTimeout(pivotDownSeconds),
            Commands.parallel(
                intake.runIntake(INTAKE_ASSIST_INTAKE_DUTY),
                indexer.runIndexer(INTAKE_ASSIST_INDEXER_DUTY),
                feeder.runFeeder(INTAKE_ASSIST_FEEDER_DUTY),
                shooter.shootRps(0.0, -25.0)
            ).withTimeout(autoIntakeSecondsAfterPivotDown)
        );
        NamedCommands.registerCommand("PivotDownAndRun", pivotDownAndRun);
        // Keep legacy name mapped to the new behavior so existing autos still work.
        NamedCommands.registerCommand("PivotDown", pivotDownAndRun);
        
        NamedCommands.registerCommand(
            "Intake",
            Commands.parallel(
                intake.runIntake(INTAKE_ASSIST_INTAKE_DUTY),
                indexer.runIndexer(INTAKE_ASSIST_INDEXER_DUTY),
                feeder.runFeeder(INTAKE_ASSIST_FEEDER_DUTY),
                shooter.shootRps(0.0, -25.0)
            )
        );
    }

    private void configureDashboard() {
        // Publish all autonomous selection/status values once so the keys always exist on the dashboard.
        SmartDashboard.putData(AUTO_CHOOSER_KEY, autoChooser);
        // Keep the legacy key alive so older dashboard layouts still see the same chooser.
        SmartDashboard.putData(LEGACY_AUTO_CHOOSER_KEY, autoChooser);
        SmartDashboard.putNumber(X_SHOT_BACK_RPS_KEY, 75.0);
        SmartDashboard.putNumber(X_SHOT_FRONT_RPS_KEY, 75.0);
        SmartDashboard.putString(AUTO_SELECTED_NAME_KEY, NO_AUTO_SELECTED);
        SmartDashboard.putBoolean(AUTO_SELECTED_VALID_KEY, false);
        SmartDashboard.putString(AUTO_STATUS_KEY, "NO AUTO SELECTED");
        SmartDashboard.putBoolean(AUTO_START_POSE_SEEDED_KEY, false);
    }

    private void configureAutoChooser() {
        // Use PathPlanner's built-in chooser so dashboards see the standard auto chooser shape.
    }

    private static double applyDriveDeadband(double value) {
        return MathUtil.applyDeadband(value, DRIVE_DEADBAND);
    }

    private void configureBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        // Left stick commands translation, right stick commands rotation.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftY()) * MaxSpeed)
                    .withVelocityY(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftX()) * MaxSpeed)
                    .withRotationalRate(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getRightX()) * MaxAngularRate)
            )
        );

        JOYSTICK1_CONTROLLER.leftBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        // D-pad drives fixed robot-centric directions for simple alignment moves.
        JOYSTICK1_CONTROLLER.povUp().whileTrue(drivetrain.applyRequest(() ->
            robotCentricNudge.withVelocityX(-0.5).withVelocityY(0).withRotationalRate(0))
        );
        JOYSTICK1_CONTROLLER.povDown().whileTrue(drivetrain.applyRequest(() ->
            robotCentricNudge.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
        );
        JOYSTICK1_CONTROLLER.povLeft().whileTrue(drivetrain.applyRequest(() ->
            robotCentricNudge.withVelocityX(0).withVelocityY(-0.5).withRotationalRate(0))
        );
        JOYSTICK1_CONTROLLER.povRight().whileTrue(drivetrain.applyRequest(() ->
            robotCentricNudge.withVelocityX(0).withVelocityY(0.5).withRotationalRate(0))
        );
        JOYSTICK1_CONTROLLER.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
            robotCentricRotate.withVelocityX(0).withVelocityY(0).withRotationalRate(DRIVER_SLOW_ROTATE_RATE))
        );
        JOYSTICK1_CONTROLLER.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
            robotCentricRotate.withVelocityX(0).withVelocityY(0).withRotationalRate(-DRIVER_SLOW_ROTATE_RATE))
        );

        // Reset the field-centric heading reference to the robot's current orientation.
        JOYSTICK1_CONTROLLER.start().onTrue(new InstantCommand(()->drivetrain.seedFieldCentric()));
    }

    private void configureOperatorBindings() {
        // Pivot controls
        JOYSTICK2_CONTROLLER.povUp().whileTrue(pivot.pivotUp());
        JOYSTICK2_CONTROLLER.povDown().whileTrue(pivot.pivotDown());

        // Intake balls / anti-jam
        JOYSTICK2_CONTROLLER.leftTrigger().whileTrue(
            Commands.parallel(
                intake.runIntake(INTAKE_ASSIST_INTAKE_DUTY),
                indexer.runIndexer(INTAKE_ASSIST_INDEXER_DUTY),
                feeder.runFeeder(INTAKE_ASSIST_FEEDER_DUTY),
                shooter.shootRps(0.0, -10.0)
            )
        );

        // Reverse all conveyors
        JOYSTICK2_CONTROLLER.b().whileTrue(
            Commands.parallel(
                intake.runIntake(REVERSE_INTAKE_DUTY),
                indexer.runIndexer(REVERSE_INDEXER_DUTY),
                feeder.runFeeder(REVERSE_FEEDER_DUTY)
            )
        );

        // Big shot
        JOYSTICK2_CONTROLLER.rightTrigger().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(120.0).withTimeout(SHOOTER_SPINUP_TIMEOUT_SECONDS),
                Commands.parallel(
                    shooter.shootRps(120.0),
                    intake.runIntake(SHOOT_INTAKE_DUTY),
                    indexer.runIndexer(SHOOT_INDEXER_DUTY),
                    feeder.runFeeder(SHOOT_FEEDER_DUTY)
                )
            )
        );

        // Regular shot
        JOYSTICK2_CONTROLLER.rightBumper().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(90.0).withTimeout(SHOOTER_SPINUP_TIMEOUT_SECONDS),
                Commands.parallel(
                    shooter.shootRps(90.0),
                    intake.runIntake(SHOOT_INTAKE_DUTY),
                    indexer.runIndexer(SHOOT_INDEXER_DUTY),
                    feeder.runFeeder(SHOOT_FEEDER_DUTY)
                )
            )
        );

        // SmartDashboard-programmed X shot
        JOYSTICK2_CONTROLLER.x().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(getXShotBackRps(), getXShotFrontRps()).withTimeout(SHOOTER_SPINUP_TIMEOUT_SECONDS),
                Commands.parallel(
                    shooter.shootRps(getXShotBackRps(), getXShotFrontRps()),
                    intake.runIntake(SHOOT_INTAKE_DUTY),
                    indexer.runIndexer(SHOOT_INDEXER_DUTY),
                    feeder.runFeeder(SHOOT_FEEDER_DUTY)
                )
            )
        );

        // Lob shot (back, front)
        JOYSTICK2_CONTROLLER.y().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(120.0, 40).withTimeout(SHOOTER_SPINUP_TIMEOUT_SECONDS),
                Commands.parallel(
                    shooter.shootRps(120.0, 40),
                    intake.runIntake(SHOOT_INTAKE_DUTY),
                    indexer.runIndexer(SHOOT_INDEXER_DUTY),
                    feeder.runFeeder(SHOOT_FEEDER_DUTY)
                )
            )
        );

        // Line-drive shot (back, front)
        JOYSTICK2_CONTROLLER.a().toggleOnTrue(
            Commands.sequence(
                shooter.shootRps(40, 120.0).withTimeout(SHOOTER_SPINUP_TIMEOUT_SECONDS),
                Commands.parallel(
                    shooter.shootRps(40, 120.0),
                    intake.runIntake(SHOOT_INTAKE_DUTY),
                    indexer.runIndexer(SHOOT_INDEXER_DUTY),
                    feeder.runFeeder(SHOOT_FEEDER_DUTY)
                )
            )
        );
    }

    public Command getAutonomousCommand() {
        Command selectedAuto = autoChooser.getSelected();
        if (!isAutoSelected(selectedAuto)) {
            setAutoStatus("NO AUTO SELECTED");
            DebugLog.warnThrottled(
                "auto.none_selected",
                "No autonomous selected; running no-op command.",
                5.0
            );
            return Commands.none();
        }
        try {
            String selectedAutoName = getSelectedAutoName(selectedAuto);
            setAutoStatus("AUTO READY: " + selectedAutoName);
            DebugLog.info("Auto selected: " + selectedAutoName);
            return selectedAuto;
        } catch (Exception ex) {
            setAutoStatus("AUTO BUILD FAILED");
            DebugLog.error(
                "Failed to get selected autonomous command.",
                ex.getStackTrace()
            );
            return Commands.none();
        }
    }

    public void updateAutoSelectionDashboard() {
        Command selectedAuto = autoChooser.getSelected();
        boolean autoSelected = isAutoSelected(selectedAuto);

        SmartDashboard.putString(
            AUTO_SELECTED_NAME_KEY,
            autoSelected ? getSelectedAutoName(selectedAuto) : NO_AUTO_SELECTED
        );
        SmartDashboard.putBoolean(AUTO_SELECTED_VALID_KEY, autoSelected);
        if (!autoSelected) {
            setAutoStatus("NO AUTO SELECTED");
            SmartDashboard.putBoolean(AUTO_START_POSE_SEEDED_KEY, false);
        }
    }

    public boolean seedPoseFromSelectedAuto() {
        Command selectedAuto = autoChooser.getSelected();
        String selectedAutoName = getSelectedAutoName(selectedAuto);
        if (!isAutoSelected(selectedAuto)) {
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
            // Use the auto's declared starting pose so odometry matches the selected routine.
            PathPlannerAuto auto = (PathPlannerAuto) selectedAuto;
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

            // PathPlanner start poses are stored from the blue-side perspective and flipped when needed.
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

    private boolean isAutoSelected(Command selectedAuto) {
        return selectedAuto instanceof PathPlannerAuto;
    }

    private String getSelectedAutoName(Command selectedAuto) {
        return selectedAuto != null ? selectedAuto.getName() : NO_AUTO_SELECTED;
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
        // These standard deviations configure how much the drivetrain estimator trusts odometry vs vision.
        return new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants, 0,
            VecBuilder.fill(RobotMap.kPositionStdDevX, RobotMap.kPositionStdDevY, Units.degreesToRadians(RobotMap.kPositionStdDevTheta)),
            VecBuilder.fill(RobotMap.kVisionStdDevX, RobotMap.kVisionStdDevY, Units.degreesToRadians(RobotMap.kVisionStdDevTheta)),
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
        );
    }
}
