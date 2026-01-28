package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class ShooterTwo extends SubsystemBase {
    // Motor controllers
    private final TalonFX topShooter;
    private final TalonFX bottomShooter;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean goingOut = false;
    private boolean goingIn = false;
    
    public ShooterTwo() {
        topShooter = new TalonFX(RobotMap.TOP_SHOOTER_TWO_MOTOR_ID);
        bottomShooter = new TalonFX(RobotMap.BOTTOM_SHOOTER_TWO_MOTOR_ID);

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
    public Command Top_Shooter_Out() {
        return this.run(() -> {
            topShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_TOP_OUT_SPEED));
            goingOut = true;
            goingIn = false;
        }).finallyDo(() -> stopMotors());
    }
    // Shooter in if needed

    /**
     *  Moves the top Shooter Clockwise.
     */

    // public Command Top_Shooter_In() {
    //     return this.run(() -> {
    //         topShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_TOP_IN_SPEED));
    //         goingIn = true;
    //         goingOut = false;
    //     }).finallyDo(() -> stopMotors());
    // }
    /**
     * Moves the bottom Shooter Counter Clockwise.
     */
    public Command Bottom_Shooter_Out() {
        return this.run(() -> {
            bottomShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_BOTTOM_OUT_SPEED));
            goingIn = false;
            goingOut = true;
        }).finallyDo(() -> stopMotors());
    }

    // Shooter in if needed

    /**
     * Moves the bottom Shooter Clockwise.
     */

    // public Command Bottom_Shooter_In() {
    //     return this.run(() -> {
    //         bottomShooter.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_SHOOTER_BOTTOM_IN_SPEED));
    //         goingIn = true;
    //         goingOut = false;
    //     }).finallyDo(() -> stopMotors());
    // }

    public void stopMotors() {
        topShooter.stopMotor();
        bottomShooter.stopMotor();
        goingOut = false;
        goingIn = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter Going Out?", goingOut);
        SmartDashboard.putBoolean("Shooter Going In?", goingIn);
    } 
}