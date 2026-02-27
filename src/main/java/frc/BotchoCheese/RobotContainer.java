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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.BotchoCheese.Commands.RotateToTag;
import frc.BotchoCheese.Subsystems.Shooter;
import frc.BotchoCheese.Subsystems.Climber;
import frc.BotchoCheese.Subsystems.Intake;
import frc.BotchoCheese.Commands.StrafeToTag;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Constants.TunerConstants;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.Feeder;


public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
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

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        NamedCommands.registerCommand("Shoot", shooter.shoot());
        NamedCommands.registerCommand("Climb", climber.climberUp());
        NamedCommands.registerCommand("IntakeOn", intake.intakeIn());

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            //TODO: May be cause of sensitivity
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-JOYSTICK1_CONTROLLER.getLeftY() * MaxSpeed * getRobotSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-JOYSTICK1_CONTROLLER.getLeftX() * MaxSpeed * getRobotSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-JOYSTICK1_CONTROLLER.getRightX() * MaxAngularRate * getRobotYawSpeed()) // Drive counterclockwise with negative X (left)
            )
        );

        JOYSTICK1_CONTROLLER.a().whileTrue(drivetrain.applyRequest(() -> brake));
        JOYSTICK1_CONTROLLER.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-JOYSTICK1_CONTROLLER.getLeftY(), -JOYSTICK1_CONTROLLER.getLeftX()))
        ));

        JOYSTICK1_CONTROLLER.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        JOYSTICK1_CONTROLLER.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // reset the field-centric heading on left bumper press
        JOYSTICK1_CONTROLLER.leftBumper().onTrue(new InstantCommand(()->drivetrain.seedFieldCentric()));
        // JOYSTICK1_CONTROLLER.leftBumper().onTrue(new InstantCommand(() -> gyro.setYaw(0)));
        // JOYSTICK1_CONTROLLER.x().onTrue(Commands.sequence(new RotateToTag(drivetrain, 0), new StrafeToTag(drivetrain, 0.5)));
        // TODO Do Left Bumper for StrafeToTag command
        // Pass in your drivetrain and the offset you want (e.g., 0.5 meters)
        JOYSTICK1_CONTROLLER.rightBumper().onTrue(new StrafeToTag(drivetrain, 0.5));
        
        // Correctly binding the shoot command
        JOYSTICK1_CONTROLLER.y().whileTrue(shooter.shoot());

        // Binding the feeder to the X button
        JOYSTICK1_CONTROLLER.x().whileTrue(feeder.runFeeder());

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    
    public static double getRobotSpeed() {
        
        return JOYSTICK1_CONTROLLER.getLeftTriggerAxis() >= 0.25 ? 0.1 : 1.0;
    // return 0.7;
    }

    public static double getRobotYawSpeed() {
        
        return JOYSTICK1_CONTROLLER.getLeftTriggerAxis() >= 0.25 ? 0.1 : 0.7*(1.0/0.9);
    // return 0.7;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        System.out.println(autoChooser.getSelected().getName());
        return autoChooser.getSelected();
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