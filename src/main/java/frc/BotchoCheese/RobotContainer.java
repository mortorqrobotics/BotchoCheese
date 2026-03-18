// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchoCheese;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BotchoCheese.Commands.RotateToTag;
import frc.BotchoCheese.Subsystems.Shooter;
import frc.BotchoCheese.Subsystems.Climber;
import frc.BotchoCheese.Subsystems.Intake;
import frc.BotchoCheese.Subsystems.Indexer;
import frc.BotchoCheese.Commands.StrafeToTag;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Constants.TunerConstants;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.Feeder;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class RobotContainer {
    private static final String DEFAULT_AUTO_NAME = "Auto 1 (Default)";
    private static final String AUTO_CHOOSER_KEY = "Auto Mode";
    private static final String PATHPLANNER_AUTO_FOLDER = "pathplanner/autos";

    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
    public static boolean pivotIsUp = true;
    public static boolean hoodIsDown = true;
    
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final static CommandXboxController JOYSTICK1_CONTROLLER = new CommandXboxController(0);
    private final static CommandXboxController JOYSTICK2_CONTROLLER = new CommandXboxController(1);
    public final static CommandSwerveDrivetrain drivetrain = createDrivetrain();
    public static Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_ID);

    // Initializing the Shooter subsystem here so it persists
    public final Shooter shooter = new Shooter();

    // Initializing the Feeder subsystem
    public final Feeder feeder = new Feeder();

    public final Climber climber = new Climber();

    public final Intake intake = new Intake();

    public final Indexer indexer = new Indexer();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        NamedCommands.registerCommand("Shoot", shooter.shoot());
        NamedCommands.registerCommand("Climb", climber.automaticClimberUp());
        NamedCommands.registerCommand("IntakeOn", intake.startIntake());
        // NamedCommands.registerCommand("IntakeOff", intake.stopIntake());

        autoChooser = new SendableChooser<>();
        configureAutoChooser();
        SmartDashboard.putData(AUTO_CHOOSER_KEY, autoChooser);

        configureBindings();
    }

    private void configureAutoChooser() {
        List<String> autoNames = getAutoNamesFromDeploy();

        if (autoNames.isEmpty()) {
            autoChooser.setDefaultOption("Do Nothing", Commands.none());
            DriverStation.reportWarning("No PathPlanner autos found in deploy/pathplanner/autos", false);
            return;
        }

        String defaultAuto = autoNames.contains(DEFAULT_AUTO_NAME) ? DEFAULT_AUTO_NAME : autoNames.get(0);
        autoChooser.setDefaultOption(defaultAuto, AutoBuilder.buildAuto(defaultAuto));

        for (String autoName : autoNames) {
            if (!autoName.equals(defaultAuto)) {
                autoChooser.addOption(autoName, AutoBuilder.buildAuto(autoName));
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

    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        /*drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftY()) * MaxSpeed)
                    .withVelocityY(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftX()) * MaxSpeed)
                    .withRotationalRate(-applyDriveDeadband(JOYSTICK1_CONTROLLER.getRightX()) * MaxAngularRate)
            )
        ); */
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double velocityX = -applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftY()) * MaxSpeed;
                double velocityY = -applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftX()) * MaxSpeed;
                double rotationalRate = -applyDriveDeadband(JOYSTICK1_CONTROLLER.getRightX()) * MaxAngularRate;

                SmartDashboard.putBoolean("Driver Controller Connected", JOYSTICK1_CONTROLLER.getHID().isConnected());
                SmartDashboard.putNumber("Driver Left Y", JOYSTICK1_CONTROLLER.getLeftY());
                SmartDashboard.putNumber("Driver Left X", JOYSTICK1_CONTROLLER.getLeftX());
                SmartDashboard.putNumber("Driver Right X", JOYSTICK1_CONTROLLER.getRightX());
                SmartDashboard.putBoolean(
                    "Driver Controller Active",
                    Math.abs(velocityX) > 0.0 || Math.abs(velocityY) > 0.0 || Math.abs(rotationalRate) > 0.0
                );

                return drive.withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(rotationalRate);
            })
        );

        new Trigger(this::isDriverControllerActive)
            .onTrue(Commands.runOnce(() -> DriverStation.reportWarning("Driver joystick movement detected", false)));
        // JOYSTICK1_CONTROLLER.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // JOYSTICK1_CONTROLLER.leftBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // JOYSTICK1_CONTROLLER.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // JOYSTICK1_CONTROLLER.a().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // JOYSTICK1_CONTROLLER.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // JOYSTICK1_CONTROLLER.x().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // Controller 1
        JOYSTICK1_CONTROLLER.x().whileTrue(drivetrain.applyRequest(() -> brake));
        // JOYSTICK1_CONTROLLER.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-JOYSTICK1_CONTROLLER.getLeftY(), -JOYSTICK1_CONTROLLER.getLeftX()))
        // ));

        JOYSTICK1_CONTROLLER.povUp().whileTrue(intake.pivotUp());
        JOYSTICK1_CONTROLLER.povDown().whileTrue(intake.pivotDown());
        JOYSTICK1_CONTROLLER.povLeft().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.25))
        );
        JOYSTICK1_CONTROLLER.povRight().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.25))
        );
        
        JOYSTICK1_CONTROLLER.a().toggleOnTrue(createFullIntakeToShooterCommand());

        JOYSTICK1_CONTROLLER.leftBumper().whileTrue(climber.manualClimberUp());
        JOYSTICK1_CONTROLLER.rightBumper().whileTrue(climber.manualClimberDown());

        // JOYSTICK1_CONTROLLER.leftTrigger().whileTrue(shooter.hoodUp());
        // JOYSTICK1_CONTROLLER.rightTrigger().whileTrue(shooter.hoodDown());

        // reset the field-centric heading on menu button press
        JOYSTICK1_CONTROLLER.start().onTrue(new InstantCommand(()->drivetrain.seedFieldCentric()));

        JOYSTICK1_CONTROLLER.rightTrigger().onTrue(new RotateToTag(drivetrain, 0));

        // reset the field-centric heading on menu button press
        JOYSTICK1_CONTROLLER.start().onTrue(new InstantCommand(()->drivetrain.seedFieldCentric()));

        //Controller 2
        // Indexer + Feeder + Shooter
        JOYSTICK2_CONTROLLER.rightTrigger().whileTrue(Commands.waitSeconds(0.5).andThen(feeder.runFeeder()));
        JOYSTICK2_CONTROLLER.rightTrigger().whileTrue(Commands.waitSeconds(0.5).andThen(indexer.indexerOn()));
        JOYSTICK2_CONTROLLER.rightTrigger().whileTrue(shooter.shoot());

        // Feeder Reverse
        JOYSTICK2_CONTROLLER.a().whileTrue(feeder.reverseFeeder());

        // Hood Up-PPAD Up/Down-DPAD Down
        // JOYSTICK2_CONTROLLER.povUp().whileTrue(shooter.hoodUp());
        // JOYSTICK2_CONTROLLER.povDown().whileTrue(shooter.hoodDown());

        // Intake + Indexer
        JOYSTICK2_CONTROLLER.leftTrigger().whileTrue(intake.startIntake());
        JOYSTICK2_CONTROLLER.leftTrigger().whileTrue(indexer.indexerOn());

        // Shooter
        JOYSTICK2_CONTROLLER.leftBumper().whileTrue(shooter.shoot());

        // Feeder
        JOYSTICK2_CONTROLLER.b().whileTrue(feeder.runFeeder());

        // Intake
        JOYSTICK2_CONTROLLER.x().whileTrue(intake.startIntake());

        // // Hood Auto
        // JOYSTICK2_CONTROLLER.a().onTrue(
        //     Commands.either(
        //         shooter.hoodDown().andThen(Commands.runOnce(() -> hoodIsDown = true)),
        //         shooter.hoodUp().andThen(Commands.runOnce(() -> hoodIsDown = false)),
        //         () -> hoodIsDown
        //     )
        // );

        // Indexer
        JOYSTICK2_CONTROLLER.y().whileTrue(indexer.indexerOn());

        // Intake Pivot Up-DPAD Left/Down-DPAD Right
        JOYSTICK2_CONTROLLER.povUp().whileTrue(intake.pivotDown().andThen(Commands.runOnce(() -> pivotIsUp = false)));
        JOYSTICK2_CONTROLLER.povDown().whileTrue(intake.pivotUp().andThen(Commands.runOnce(() -> pivotIsUp = true)));

        // Intake Auto Pivot
        JOYSTICK2_CONTROLLER.rightBumper().onTrue(

            Commands.either(
                intake.setPivotDown().andThen(Commands.runOnce(() -> pivotIsUp = false)),
                intake.setPivotUp().andThen(Commands.runOnce(() -> pivotIsUp = true)),
                () -> pivotIsUp
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    private static double applyDriveDeadband(double value) {
        return MathUtil.applyDeadband(value, 0.1);
    }

    private boolean isDriverControllerActive() {
        return Math.abs(applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftY())) > 0.0
            || Math.abs(applyDriveDeadband(JOYSTICK1_CONTROLLER.getLeftX())) > 0.0
            || Math.abs(applyDriveDeadband(JOYSTICK1_CONTROLLER.getRightX())) > 0.0;
    }

    private Command createFullIntakeToShooterCommand() {
        return Commands.parallel(
           // intake.routineIntakeOn(),
            indexer.indexerOn(),
            feeder.runFeeder(),
            shooter.shoot()
        );
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        Command selected = autoChooser.getSelected();
        if (selected == null) {
            DriverStation.reportWarning("No autonomous selected; running no-op command.", false);
            return Commands.none();
        }
        System.out.println(selected.getName());
        return selected;
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
