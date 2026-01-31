package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.BotchoCheese.Robot;
//import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class ShooterTwo extends SubsystemBase {
    // Motor controllers
    private final TalonFX topTwoShooter;
    private final TalonFX bottomTwoShooter;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean goingLeft = false;
    private boolean goingRight = false;
    
    public ShooterTwo() {
        topTwoShooter = new TalonFX(RobotMap.TOP_SHOOTER_TWO_MOTOR_ID);
        bottomTwoShooter = new TalonFX(RobotMap.BOTTOM_SHOOTER_TWO_MOTOR_ID);

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID Values
        // in init function, set slot 0 gains
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = RobotMap.SHOOTER_P_VALUE;
        slot0Configs.kI = RobotMap.SHOOTER_I_VALUE;
        slot0Configs.kD = RobotMap.SHOOTER_D_VALUE;
        config.Slot0 = slot0Configs;
        //https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
        
        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits = currentLimits;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        topTwoShooter.getConfigurator().apply(config);
        bottomTwoShooter.getConfigurator().apply(config);
    }

    /**
     * Moves the top Shooter counter Clockwise.
     */
    public Command topTwoShooterTurnLeft() {
        return this.run(() -> {
            topTwoShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_TOP_OUT_SPEED));
            goingLeft = true;
            goingRight = false;
        }).finallyDo(() -> stopMotors());
    }
    // Shooter in if needed

    // public Command topTwoShooterTurnRight() {
    //     return this.run(() -> {
    //         topTwoShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_TOP_IN_SPEED));
    //         goingRight = true;
    //         goingLeft = false;
    //     }).finallyDo(() -> stopMotors());
    // }
    /**
     * Moves the bottom Shooter Counter Clockwise.
     */
    public Command bottomTwoShooterTurnLeft() {
        return this.run(() -> {
            bottomTwoShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_BOTTOM_OUT_SPEED));
            goingLeft = true;
            goingRight = false;
        }).finallyDo(() -> stopMotors());
    }

    // Shooter in if needed

    // public Command bottomTwoShooterTurnRight() {
    //     return this.run(() -> {
    //         bottomTwoShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_BOTTOM_IN_SPEED));
    //         goingRight = true;
    //         goingLeft = false;
    //     }).finallyDo(() -> stopMotors());
    // }

    public Command shoot() {
        return this.run(() -> {
            topTwoShooter.set(-RobotMap.SHOOTER_SPEED);
            bottomTwoShooter.set(-RobotMap.SHOOTER_SPEED);
        }).finallyDo(() -> stopMotors());
    }

    public void stopMotors() {
        topTwoShooter.stopMotor();
        bottomTwoShooter.stopMotor();
        goingLeft = false;
        goingRight = false;
    }

    /**
     * Update shooter speed based on distance from target
     */
    public void updateSpeed() {
        //TODO: Find what the new version of this function is in the LimelightHelpers file
        //double distance = LimelightHelpers.getDistance();
        //speed = calcSpeed(distance);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("kP", RobotMap.SHOOTER_P_VALUE);
        SmartDashboard.putNumber("kI", RobotMap.SHOOTER_I_VALUE);
        SmartDashboard.putNumber("kD", RobotMap.SHOOTER_D_VALUE);
        SmartDashboard.putBoolean("Shooter Turning Left?", goingLeft);
        SmartDashboard.putBoolean("Shooter Turning Right?", goingRight);

        SmartDashboard.putNumber("Top Two Shooter Battery Draw", topTwoShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Top Two Shooter Motor Draw", topTwoShooter.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Two Shooter Battery Draw", bottomTwoShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Two Shooter Motor Draw", bottomTwoShooter.getStatorCurrent().getValueAsDouble());
    } 
}