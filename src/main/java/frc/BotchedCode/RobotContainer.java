// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.BotchedCode.Commands.RotateToTag;
import frc.BotchedCode.Commands.StrafeToTag;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Constants.TunerConstants;
import frc.BotchedCode.Constants.TunerConstantsOld;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;



public class RobotContainer {
    public CANdle candle;
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

    private final static CommandXboxController joystick = new CommandXboxController(0);
    
    public final static CommandSwerveDrivetrain drivetrain = createDrivetrain();
    public static Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_ID);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        candle = new CANdle(0);
       
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.stripType = LEDStripType.GRB;
        config.v5Enabled = true;
        config.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        config.brightnessScalar = 1;
        candle.configAllSettings(config, 100);
        candle.configLEDType(LEDStripType.GRB); //just added this after cd post

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * getRobotSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * getRobotSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * getRobotYawSpeed()) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.x().onTrue(Commands.sequence(new RotateToTag(drivetrain, 0), new StrafeToTag(drivetrain, 0.5)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    
    public static double getRobotSpeed() {
        
        return joystick.getLeftTriggerAxis() >= 0.25 ? 0.1 : 1.0;
    // return 0.7;
    }

    public static double getRobotYawSpeed() {
        
        return joystick.getLeftTriggerAxis() >= 0.25 ? 0.1 : 0.7*(1.0/0.9);
    // return 0.7;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        System.out.println(autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }

    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            TunerConstantsOld.DrivetrainConstants, 0,
            VecBuilder.fill(RobotMap.kPositionStdDevX, RobotMap.kPositionStdDevY, Units.degreesToRadians(RobotMap.kPositionStdDevTheta)),
            VecBuilder.fill(RobotMap.kVisionStdDevX, RobotMap.kVisionStdDevY, Units.degreesToRadians(RobotMap.kVisionStdDevTheta)),
            TunerConstantsOld.FrontLeft, TunerConstantsOld.FrontRight, TunerConstantsOld.BackLeft, TunerConstantsOld.BackRight
        );
    }
}