package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class Shooter extends SubsystemBase {
    // Motor controllers
    private final TalonFX leftShooter;
    private final TalonFX middleShooter;
    private final TalonFX rightShooter;
    private final TalonFXS hood;
    private final DutyCycleEncoder hoodEncoder;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);
    private final MotionMagicVoltage hoodPositionRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage shooterVelocityRequest = new MotionMagicVelocityVoltage(0);

    private boolean shooterTurning = false;
    private boolean hoodDown = true;
    
    public Shooter() {
        leftShooter = new TalonFX(RobotMap.LEFT_SHOOTER_MOTOR_ID);
        middleShooter = new TalonFX(RobotMap.MIDDLE_SHOOTER_MOTOR_ID);
        rightShooter = new TalonFX(RobotMap.RIGHT_SHOOTER_MOTOR_ID);
        hood = new TalonFXS(RobotMap.HOOD_MOTOR_ID);
        hoodEncoder = new DutyCycleEncoder(RobotMap.HOOD_THROUGHBORE_DIO);

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();
        hoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

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

        // PID Hood Values
        // in init function, set slot 0 gains
        Slot0Configs hoodSlot0 = new Slot0Configs();
        hoodSlot0.kS = RobotMap.SHOOTER_S_VALUE;
        hoodSlot0.kV = RobotMap.SHOOTER_V_VALUE;
        hoodSlot0.kA = RobotMap.SHOOTER_A_VALUE;
        hoodSlot0.kP = RobotMap.HOOD_P_VALUE;
        hoodSlot0.kI = RobotMap.HOOD_I_VALUE;
        hoodSlot0.kD = RobotMap.HOOD_D_VALUE;
        hoodConfig.Slot0 = hoodSlot0;

        MotionMagicConfigs shooterMotionMagicConfigs = config.MotionMagic;
        shooterMotionMagicConfigs.MotionMagicAcceleration = RobotMap.SHOOTER_ACCELERATION;
        shooterMotionMagicConfigs.MotionMagicJerk = RobotMap.SHOOTER_JERK;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = RobotMap.HOOD_CRUISE_VELOCITY; // Target cruise velocity of 40 rps
        motionMagicConfigs.MotionMagicAcceleration = RobotMap.HOOD_ACCELERATION; // Target acceleration of 80 rps/s
        motionMagicConfigs.MotionMagicJerk = RobotMap.HOOD_JERK; // Target jerk of 800 rps/s/s
        hoodConfig.MotionMagic = motionMagicConfigs;
        
        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits = currentLimits;
        hoodConfig.CurrentLimits = currentLimits;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftShooter.getConfigurator().apply(config);
        middleShooter.getConfigurator().apply(config);
        rightShooter.getConfigurator().apply(config);
        hood.getConfigurator().apply(hoodConfig);

        //System.out.println("Shooter constructed=====================");
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

    private double getHoodEncoderPositionRotations() {
        double adjustedRotations = hoodEncoder.get() - RobotMap.HOOD_THROUGHBORE_OFFSET_ROT;
       // System.out.println("shooter getHoodEncoderPositionRotations executed=====================");
        return MathUtil.inputModulus(adjustedRotations, 0.0, 1.0);
    }

    private boolean atHoodUpperLimit() {
        // System.out.println("shooter atHoodUpperLimit executed=====================");
        return getHoodEncoderPositionRotations()
            >= RobotMap.HOOD_MAX_ROT - RobotMap.HOOD_LIMIT_TOLERANCE_ROT;
    }

    private boolean atHoodLowerLimit() {
        //System.out.println("shooter atLowerLimit executed=====================");
        return getHoodEncoderPositionRotations()
            <= RobotMap.HOOD_MIN_ROT + RobotMap.HOOD_LIMIT_TOLERANCE_ROT;
    }

    public Command hoodUp() {
        //System.out.println("shooter hoodUp executed=====================");
        return this.run(() -> {
            if (!atHoodUpperLimit()) {
                hood.setControl(m_output.withOutput(RobotMap.HOOD_SPEED));
            } else {
                hood.stopMotor();
            }
            hoodDown = false;
        }).until(this::atHoodUpperLimit).finallyDo(() -> hood.stopMotor());
    }

    public Command hoodDown() {
        //System.out.println("shooter hoodDown executed=====================");
        return this.run(() -> {
            if (!atHoodLowerLimit()) {
                hood.setControl(m_output.withOutput(-RobotMap.HOOD_SPEED));
            } else {
                hood.stopMotor();
            }
            hoodDown = true;
        }).until(this::atHoodLowerLimit).finallyDo(() -> hood.stopMotor());
    }

    public Command shoot() {
        // We use startEnd so it automatically stops motors when the command finishes (button release)
        return this.startEnd(
            // When command starts/runs:
            () -> {
                //System.out.println("Kapoooooooooooooow!");
                hood.setControl(hoodPositionRequest.withPosition(RobotMap.HOOD_POSITION));
                leftShooter.setControl(shooterVelocityRequest.withVelocity(RobotMap.SHOOTER_TARGET_RPS));
                middleShooter.setControl(shooterVelocityRequest.withVelocity(RobotMap.SHOOTER_TARGET_RPS));
                rightShooter.setControl(shooterVelocityRequest.withVelocity(RobotMap.SHOOTER_TARGET_RPS));
            },
            // When command ends:
            () -> {
                stopMotors();
            }
        );
    }

    public void stopMotors() {
        leftShooter.stopMotor();
        middleShooter.stopMotor();
        rightShooter.stopMotor();
        hood.stopMotor();
        shooterTurning = false;
        //System.out.println("shooter stopMotors executed=====================");
    }

    /**
     * Update shooter speed based on distance from target
     */
    public void updateSpeed() {
        RobotMap.SHOOTER_SPEED = 0.85;
    }

    public void updateHoodPosition() {
        //System.out.println("shooter updateHoodPosition executed=====================");
        LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(RobotMap.LIMELIGHT_NAME);
        if (rawFiducials.length == 0) {
            //System.out.println("shooter updateHoodPosition NO RAW FIDUCIALS executed=====================");
            return;
        }
        double distance = rawFiducials[0].distToRobot;

       if(distance <= RobotMap.SHORT_DISTANCE_THRESHOLD) {
            RobotMap.HOOD_POSITION = 0.25; // TODO
            //System.out.println("shooter updateHoodPosition short executed=====================");
       }
       else if(distance > RobotMap.SHORT_DISTANCE_THRESHOLD && distance <= RobotMap.MEDIUM_DISTANCE_THRESHOLD) {
            RobotMap.HOOD_POSITION = 0.5; // TODO
            //System.out.println("shooter updateHoodPosition medium executed=====================");
       }
       else {
            RobotMap.HOOD_POSITION = 0.75; // TODO
            //System.out.println("shooter updateHoodPosition long executed=====================");
       }
    }

    @Override
    public void periodic() {
        updateHoodPosition();
        updateSpeed();

        SmartDashboard.putNumber("kP", RobotMap.SHOOTER_P_VALUE);
        SmartDashboard.putNumber("kI", RobotMap.SHOOTER_I_VALUE);
        SmartDashboard.putNumber("kD", RobotMap.SHOOTER_D_VALUE);
        SmartDashboard.putBoolean("Shooter Turning?", shooterTurning);
        SmartDashboard.putBoolean("Hood Going Down?", hoodDown);
        
        SmartDashboard.putNumber("Shooter Battery Draw", leftShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor Draw", leftShooter.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood Battery Draw", hood.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood Motor Draw", hood.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood Throughbore Rot", getHoodEncoderPositionRotations());
        SmartDashboard.putBoolean("Hood At Upper Limit", atHoodUpperLimit());
        SmartDashboard.putBoolean("Hood At Lower Limit", atHoodLowerLimit());
    } 
}
