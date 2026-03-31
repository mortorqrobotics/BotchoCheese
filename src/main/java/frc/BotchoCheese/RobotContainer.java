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
import frc.BotchoCheese.Commands.StrafeToTag;
import frc.BotchoCheese.Commands.RotateToTag;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Constants.TunerConstants;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Subsystems.Feeder;
import frc.BotchoCheese.Subsystems.Indexer;
import frc.BotchoCheese.Subsystems.Intake;
import frc.BotchoCheese.Subsystems.Shooter;
import frc.BotchoCheese.Subsystems.Pivot;


public class RobotContainer {
    private static final String DEFAULT_AUTO_NAME = "Auto 1 (Default)";
    private static final String AUTO_CHOOSER_KEY = "Auto Mode";
    private static final String PATHPLANNER_AUTO_FOLDER = "pathplanner/autos";

    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
    public static boolean pivotIsUp = true;
    
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 
            // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final static CommandXboxController JOYSTICK1_CONTROLLER = new CommandXboxController(0);
    private final static CommandXboxController JOYSTICK2_CONTROLLER = new CommandXboxController(1);
    public final static CommandSwerveDrivetrain drivetrain = createDrivetrain();
    public static Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_ID);

    // Initializing the Shooter subsystem here so it persists
    public final Shooter shooter = new Shooter();

    // Initializing the Feeder subsystem
    public final Feeder feeder = new Feeder();


    public final Intake intake = new Intake();

    public final Indexer indexer = new Indexer();

     public final Pivot pivot = new Pivot();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        NamedCommands.registerCommand("Shoot", 
            Commands.sequence(
              //  Commands.parallel(feeder.reverseFeeder().withTimeout(0.5), indexer.reverseIndexer().withTimeout(0.25), shooter.shootRps(80).withTimeout(0.5), intake.startIntake().withTimeout(0.5))).andThen(Commands.parallel(shooter.shootRps(80).withTimeout(2), feeder.runFeeder().withTimeout(2), indexer.indexerOn().withTimeout(2), intake.startIntake().withTimeout(2))                
            )
        );
        //NamedCommands.registerCommand("PivotDown", intake.pivotDown().withTimeout(1.5).andThen(intake.startIntake().withTimeout(1.5)));
        // NamedCommands.registerCommand("PivotUp", intake.pivotDown().withTimeout(3));
        NamedCommands.registerCommand("IntakeOn", intake.runIntake(0.5));
        //NamedCommands.registerCommand("IntakeOff", new InstantCommand(()->intake.stopIntake()));

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
        
        JOYSTICK1_CONTROLLER.start().onTrue(new InstantCommand(()->drivetrain.seedFieldCentric()));

        JOYSTICK1_CONTROLLER.leftTrigger().whileTrue(new StrafeToTag(drivetrain));
        JOYSTICK1_CONTROLLER.rightTrigger().whileTrue(new RotateToTag(drivetrain, 0));

        //Controller 2 Bindings

        //Pivot Control
        JOYSTICK2_CONTROLLER.povUp().whileTrue(pivot.pivotUp());
        JOYSTICK2_CONTROLLER.povDown().whileTrue(pivot.pivotDown());

        //INTAKE BALLs
        JOYSTICK2_CONTROLLER.leftTrigger().toggleOnTrue(
        Commands.parallel(
        intake.runIntake(-0.5),
        indexer.runIndexer(-0.5),
        feeder.runFeeder(-0.5)
     
    )
);

//Shooter sequence
JOYSTICK2_CONTROLLER.rightTrigger().toggleOnTrue(
    Commands.sequence(
        // Spin up shooter for 2s
        shooter.shootRps(90).withTimeout(1.0),

        // Then run everything continuously until toggled off
        Commands.parallel(
            shooter.shootRps(90),
            feeder.runFeeder(0.5),
            indexer.runIndexer(0.5),
            intake.runIntake(0.5),
            pivot.pivotUpToRotations(4)
        )
    )
);

//Reverse everything
JOYSTICK2_CONTROLLER.b().whileTrue(
    Commands.parallel(
        feeder.runFeeder(-0.5),
        indexer.runIndexer(-0.5),
        intake.runIntake(-0.5)
    )
);

    }
    
    private static double applyDriveDeadband(double value) {
        return MathUtil.applyDeadband(value, 0.1);
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
