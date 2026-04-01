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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

public class RobotContainer {
    // Auto chooser/dashboard
    private static final String DEFAULT_AUTO_NAME = "Auto 1 (Default)";
    private static final String AUTO_CHOOSER_KEY = "Auto Mode";
    private static final String PATHPLANNER_AUTO_FOLDER = "pathplanner/autos";

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
        NamedCommands.registerCommand("Shoot", Commands.sequence());
        NamedCommands.registerCommand("IntakeOn", intake.runIntake(0.5));
    }

    private void configureDashboard() {
        SmartDashboard.putData(AUTO_CHOOSER_KEY, autoChooser);
        SmartDashboard.putData("Zero Pivot Encoder", new InstantCommand(pivot::zeroPivotEncoder, pivot));
    }

    private void configureAutoChooser() {
        List<String> autoNames = getAutoNamesFromDeploy();

        if (autoNames.isEmpty()) {
            autoChooser.setDefaultOption("Do Nothing", "Do Nothing");
            DriverStation.reportWarning("No PathPlanner autos found in deploy/pathplanner/autos", false);
            return;
        }

        String defaultAuto = autoNames.contains(DEFAULT_AUTO_NAME) ? DEFAULT_AUTO_NAME : autoNames.get(0);
        autoChooser.setDefaultOption(defaultAuto, defaultAuto);

        for (String autoName : autoNames) {
            if (!autoName.equals(defaultAuto)) {
                autoChooser.addOption(autoName, autoName);
            }
        }
    }

    private List<String> getAutoNamesFromDeploy() {
        Path autoFolder = Filesystem.getDeployDirectory().toPath().resolve(PATHPLANNER_AUTO_FOLDER);
        if (!Files.isDirectory(autoFolder)) {
            return List.of();
        }

        try (var files = Files.list(autoFolder)) {
            return files
                .filter(path -> path.toString().endsWith(".auto"))
                .map(path -> path.getFileName().toString().replaceFirst("\\.auto$", ""))
                .sorted(Comparator.naturalOrder())
                .collect(Collectors.toList());
        } catch (IOException e) {
            DriverStation.reportError("Failed to read PathPlanner autos: " + e.getMessage(), e.getStackTrace());
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
                    .withRotationalRate(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getRightX()) * MaxAngularRate)
            )
        );

        JOYSTICK1_CONTROLLER.x().whileTrue(drivetrain.applyRequest(() -> brake));

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
                shooter.shootRps(120.0).withTimeout(1.0),
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
                shooter.shootRps(75.0).withTimeout(1.0),
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
                shooter.shootRps(120.0, 6.0).withTimeout(1.0),
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
                shooter.shootRps(6.0, 120.0).withTimeout(1.0),
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
        if (selectedAutoName == null || selectedAutoName.equals("Do Nothing")) {
            DriverStation.reportWarning("No autonomous selected; running no-op command.", false);
            return Commands.none();
        }
        System.out.println(selectedAutoName);
        return AutoBuilder.buildAuto(selectedAutoName);
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
