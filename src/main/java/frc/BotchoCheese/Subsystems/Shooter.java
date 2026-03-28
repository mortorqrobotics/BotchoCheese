package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here
import frc.BotchoCheese.Subsystems.Intake;

public class Shooter extends SubsystemBase {
    // Motor controllers
    private final TalonFX backLeftShooter;
    private final TalonFX backRightShooter;
    private final TalonFX frontShooter;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut shooter_output = new DutyCycleOut(0);

    private boolean shooterTurning = false;

    private static int buttonPresses = 0;
    private double shooterSpeed = 0.82;

    // TODO Make sure this doesn't cause issues with RobotContainer
    public final Intake intake = new Intake();
    
    public Shooter() {
        backLeftShooter = new TalonFX(RobotMap.BACK_LEFT_SHOOTER_MOTOR_ID);
        backRightShooter = new TalonFX(RobotMap.BACK_RIGHT_SHOOTER_MOTOR_ID);
        frontShooter = new TalonFX(RobotMap.FRONT_SHOOTER_MOTOR_ID);

        buttonPresses = 0;
        shooterSpeed = 0.82;

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID Shooter Values
        // in init function, set slot 0 gains
        Slot0Configs shooterSlot0 = new Slot0Configs();
        shooterSlot0.kS = RobotMap.SHOOTER_S_VALUE;
        shooterSlot0.kV = RobotMap.SHOOTER_V_VALUE;
        shooterSlot0.kA = RobotMap.SHOOTER_A_VALUE;
        shooterSlot0.kP = RobotMap.SHOOTER_P_VALUE;
        shooterSlot0.kI = RobotMap.SHOOTER_I_VALUE;
        shooterSlot0.kD = RobotMap.SHOOTER_D_VALUE;
        config.Slot0 = shooterSlot0;
        //https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html

        // MotionMagicConfigs shooterMotionMagicConfigs = config.MotionMagic;
        // shooterMotionMagicConfigs.MotionMagicAcceleration = RobotMap.SHOOTER_ACCELERATION;
        // shooterMotionMagicConfigs.MotionMagicJerk = RobotMap.SHOOTER_JERK;
        
        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits = currentLimits;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        backLeftShooter.getConfigurator().apply(config);
        backRightShooter.getConfigurator().apply(config);
        frontShooter.getConfigurator().apply(config);
    }

    // Moves the Shooter counter Clockwise.
    
    // public Command turnShooters() {
    //     return this.run(() -> {
    //         leftShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
    //         middleShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
    //         rightShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
    //         shooterTurning = true;
    //     }).finallyDo(() -> stopMotors());
    // }

    //1. Use tx and ty to get distance from the robot and the tag
    //2. Determine whether the distance is "close," "medium," or "far" away from the tag
    //3. Change the speed accordingly for each of these situations

    // private double getHoodEncoderPositionRotations() {
    //     double adjustedRotations = hoodEncoder.get() - RobotMap.HOOD_THROUGHBORE_OFFSET_ROT;
    //     return MathUtil.inputModulus(adjustedRotations, 0.0, 1.0);
    // }

    // private boolean atHoodUpperLimit() {
    //     return getHoodEncoderPositionRotations()
    //         >= RobotMap.HOOD_MAX_ROT - RobotMap.HOOD_LIMIT_TOLERANCE_ROT;
    // }

    // private boolean atHoodLowerLimit() {
    //     return getHoodEncoderPositionRotations()
    //         <= RobotMap.HOOD_MIN_ROT + RobotMap.HOOD_LIMIT_TOLERANCE_ROT;
    // }

    // public Command hoodUp() {
    //     return this.run(() -> {
    //         if (!atHoodUpperLimit()) {
    //             hood.setControl(m_output.withOutput(-RobotMap.HOOD_SPEED));
    //         } else {
    //             hood.stopMotor();
    //         }
    //         hoodDown = false;
    //     }).until(this::atHoodUpperLimit).finallyDo(() -> hood.stopMotor());
    // }

    // public Command hoodDown() {
    //     return this.run(() -> {
    //         if (!atHoodLowerLimit()) {
    //             hood.setControl(m_output.withOutput(RobotMap.HOOD_SPEED));
    //         } else {
    //             hood.stopMotor();
    //         }
    //         hoodDown = true;
    //     }).until(this::atHoodLowerLimit).finallyDo(() -> hood.stopMotor());
    // }

    public Command shoot() {
        // We use startEnd so it automatically stops motors when the command finishes (button release)
        return this.startEnd(
            // When command starts/runs:
            () -> {
                backLeftShooter.setControl(shooter_output.withOutput(RobotMap.SHOOTER_SPEED));
                backRightShooter.setControl(shooter_output.withOutput(RobotMap.SHOOTER_SPEED));
                frontShooter.setControl(shooter_output.withOutput(RobotMap.SHOOTER_SPEED));
                shooterTurning = true;
            },
            // When command ends:
            () -> {
                stopMotors();
            }
        );
    }

    public Command reverseShoot() {
        // We use startEnd so it automatically stops motors when the command finishes (button release)
        return this.startEnd(
            // When command starts/runs:
            () -> {
                backLeftShooter.setControl(shooter_output.withOutput(-RobotMap.SHOOTER_SPEED));
                backRightShooter.setControl(shooter_output.withOutput(-RobotMap.SHOOTER_SPEED));
                frontShooter.setControl(shooter_output.withOutput(-RobotMap.SHOOTER_SPEED));
            },
            // When command ends:
            () -> {
                stopMotors();
            }
        );
    }

    public void stopMotors() {
        backLeftShooter.stopMotor();
        backRightShooter.stopMotor();
        frontShooter.stopMotor();
        shooterTurning = false;
        intake.updateShooterStatus(false);
    }

    /**
     * Update shooter speed based on distance from target
     */
    public void updateSpeed() {

        LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(RobotMap.LIMELIGHT_NAME);
        if (rawFiducials.length == 0) {
            return;
        }
        double distance = rawFiducials[0].distToRobot;
        //Short
       if(distance <= RobotMap.SHORT_DISTANCE_THRESHOLD) {
            //RobotMap.SHOOTER_SPEED = 0.25; 
            shooterSpeed = RobotMap.SHOOTER_SPEED_1;
       }
       //Medium
       else if(distance > RobotMap.SHORT_DISTANCE_THRESHOLD && distance <= RobotMap.MEDIUM_DISTANCE_THRESHOLD) {
            //RobotMap.SHOOTER_SPEED = 0.5; 
            shooterSpeed = RobotMap.SHOOTER_SPEED_2;
       }
       //Long/Regular
       else {
            //RobotMap.SHOOTER_SPEED = 0.75;
            shooterSpeed = RobotMap.SHOOTER_SPEED_3;
       }
    }

    public void cycleSpeed() {
        if(buttonPresses == 0) {
            //Set to speed 1
            shooterSpeed = RobotMap.SHOOTER_SPEED_1;
            buttonPresses = 1;
        }
        else if(buttonPresses == 1) {
            //Set to speed 2
            shooterSpeed = RobotMap.SHOOTER_SPEED_2;
            buttonPresses = 2;
        }
        else if(buttonPresses == 2) {
            //Set to speed 3
            shooterSpeed = RobotMap.SHOOTER_SPEED_3;
            buttonPresses = 0;
        }
    }

    // public void updateHoodPosition() {
    //     LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(RobotMap.LIMELIGHT_NAME);
    //     if (rawFiducials.length == 0) {
    //         return;
    //     }
    //     double distance = rawFiducials[0].distToRobot;

    //    if(distance <= RobotMap.SHORT_DISTANCE_THRESHOLD) {
    //         RobotMap.HOOD_POSITION = 0.25;
    //    }
    //    else if(distance > RobotMap.SHORT_DISTANCE_THRESHOLD && distance <= RobotMap.MEDIUM_DISTANCE_THRESHOLD) {
    //         RobotMap.HOOD_POSITION = 0.5;
    //    }
    //    else {
    //         RobotMap.HOOD_POSITION = 0.75;
    //    }
    // }

    @Override
    public void periodic() {
        //TODO!: Uncomment this if Limelights work
        //updateSpeed();

        SmartDashboard.putNumber("kP", RobotMap.SHOOTER_P_VALUE);
        SmartDashboard.putNumber("kI", RobotMap.SHOOTER_I_VALUE);
        SmartDashboard.putNumber("kD", RobotMap.SHOOTER_D_VALUE);
        SmartDashboard.putBoolean("Shooter Turning?", shooterTurning);
        
        SmartDashboard.putNumber("Shooter Battery Draw", backLeftShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor Draw", backLeftShooter.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Mode", buttonPresses);
    } 
}
