package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Robot;
import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class ShooterOne extends SubsystemBase {
    // Motor controllers
    private final TalonFX topShooter;
    private final TalonFX bottomShooter;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean goingLeft = false;
    private boolean goingRight = false;
    
    public ShooterOne() {
        topShooter = new TalonFX(RobotMap.TOP_SHOOTER_ONE_MOTOR_ID);
        bottomShooter = new TalonFX(RobotMap.BOTTOM_SHOOTER_ONE_MOTOR_ID);

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        topShooter.getConfigurator().apply(config);
        bottomShooter.getConfigurator().apply(config);
        // TODO: Implement PID Map/Values
    }

    /**
     * Moves the top Shooter counter Clockwise.
     */
    public Command topShooterTurnLeft() {
        return this.run(() -> {
            topShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_TOP_OUT_SPEED));
            goingLeft = true;
            goingRight = false;
        }).finallyDo(() -> stopMotors());
    }
    // Shooter in if needed

    // public Command topShooterTurnRight() {
    //     return this.run(() -> {
    //         topShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_TOP_IN_SPEED));
    //         goingRight = true;
    //         goingLeft = false;
    //     }).finallyDo(() -> stopMotors());
    // }
    /**
     * Moves the bottom Shooter Counter Clockwise.
     */
    public Command bottomShooterTurnLeft() {
        return this.run(() -> {
            bottomShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_BOTTOM_OUT_SPEED));
            goingLeft = true;
            goingRight = false;
        }).finallyDo(() -> stopMotors());
    }

    // Shooter in if needed

    // public Command bottomShooterTurnRight() {
    //     return this.run(() -> {
    //         bottomShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_BOTTOM_IN_SPEED));
    //         goingRight = true;
    //         goingLeft = false;
    //     }).finallyDo(() -> stopMotors());
    // }

    public void stopMotors() {
        topShooter.stopMotor();
        bottomShooter.stopMotor();
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
        SmartDashboard.putBoolean("Shooter Turning Left?", goingLeft);
        SmartDashboard.putBoolean("Shooter Turning Right?", goingRight);
    } 
}