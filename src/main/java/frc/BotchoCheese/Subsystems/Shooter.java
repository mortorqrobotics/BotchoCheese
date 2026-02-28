package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.BotchoCheese.Robot;
import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class Shooter extends SubsystemBase {
    // Motor controllers
    private final TalonFX leftShooter;
    private final TalonFX middleShooter;
    private final TalonFX rightShooter;
    private final TalonFXS hood;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean shooterTurning = false;
    private boolean hoodUp = false;
    private boolean hoodDown = false;
    
    public Shooter() {
        leftShooter = new TalonFX(RobotMap.LEFT_SHOOTER_MOTOR_ID);
        middleShooter = new TalonFX(RobotMap.MIDDLE_SHOOTER_MOTOR_ID);
        rightShooter = new TalonFX(RobotMap.RIGHT_SHOOTER_MOTOR_ID);
        hood = new TalonFXS(RobotMap.HOOD_MOTOR_ID);

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();

        // PID Shooter Values
        // in init function, set slot 0 gains
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = RobotMap.SHOOTER_P_VALUE;
        slot0Configs.kI = RobotMap.SHOOTER_I_VALUE;
        slot0Configs.kD = RobotMap.SHOOTER_D_VALUE;
        config.Slot0 = slot0Configs;
        //https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html

        // PID Hood Values
        // in init function, set slot 0 gains
        Slot0Configs slot1Configs = new Slot0Configs();
        slot1Configs.kP = RobotMap.HOOD_P_VALUE;
        slot1Configs.kI = RobotMap.HOOD_I_VALUE;
        slot1Configs.kD = RobotMap.HOOD_D_VALUE;
        hoodConfig.Slot0 = slot1Configs;
        
        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits = currentLimits;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftShooter.getConfigurator().apply(config);
        middleShooter.getConfigurator().apply(config);
        rightShooter.getConfigurator().apply(config);
        hood.getConfigurator().apply(hoodConfig);
    }

    /**
     * Moves the top Shooter counter Clockwise.
     */
    public Command turnShooter() {
        return this.run(() -> {
            leftShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
            middleShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
            rightShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
            shooterTurning = true;
        }).finallyDo(() -> stopMotors());
    }

    //1. Use tx and ty to get distance from the robot and the tag
    //2. Determine whether the distance is "close," "medium," or "far" away from the tag
    //3. Change the speed accordingly for each of these situations

    public Command hoodUp() {
        return this.run(() -> {
            hood.setControl(m_output.withOutput(RobotMap.HOOD_SPEED));
            hoodUp = true;
            hoodDown = false;
        }).finallyDo(() -> stopMotors());
    }

    public Command hoodDown() {
        return this.run(() -> {
            hood.setControl(m_output.withOutput(-RobotMap.HOOD_SPEED));   
            hoodUp = true;
            hoodDown = false;
        }).finallyDo(() -> stopMotors());
    }

    public Command shoot() {
        // We use startEnd so it automatically stops motors when the command finishes (button release)
        return this.startEnd(
            // When command starts/runs:
            () -> {
                System.out.println("Bam!");
                System.out.println("Kapoooooooooooooow!");
                leftShooter.set(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED);
                middleShooter.set(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED);
                rightShooter.set(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED);
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

       if(distance <= RobotMap.SHORT_DISTANCE_THRESHOLD) {
            RobotMap.SHOOTER_SPEED = 0.25;
       }
       else if(distance > RobotMap.SHORT_DISTANCE_THRESHOLD && distance <= RobotMap.MEDIUM_DISTANCE_THRESHOLD) {
            RobotMap.SHOOTER_SPEED = 0.5;
       }
       else {
            RobotMap.SHOOTER_SPEED = 0.75;
       }
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("kP", RobotMap.SHOOTER_P_VALUE);
        SmartDashboard.putNumber("kI", RobotMap.SHOOTER_I_VALUE);
        SmartDashboard.putNumber("kD", RobotMap.SHOOTER_D_VALUE);
        SmartDashboard.putBoolean("Shooter Turning?", shooterTurning);
        SmartDashboard.putBoolean("Hood Going Up?", hoodUp);
        SmartDashboard.putBoolean("Hood Going Down?", hoodDown);
        
        SmartDashboard.putNumber("Shooter Battery Draw", leftShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor Draw", leftShooter.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood Battery Draw", hood.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood Motor Draw", hood.getStatorCurrent().getValueAsDouble());
    } 
}

