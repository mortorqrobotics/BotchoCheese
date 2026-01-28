package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class Intake extends SubsystemBase {
    // Motor controllers
    private final TalonFX topIntake;
    private final TalonFX bottomIntake;
    private final TalonFX insideIntake;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean goingIn = false;
    private boolean goingOut = false;
    
    public Intake() {
        topIntake = new TalonFX(RobotMap.TOP_INTAKE_MOTOR_ID);
        bottomIntake = new TalonFX(RobotMap.BOTTOM_INTAKE_MOTOR_ID);
        insideIntake = new TalonFX(RobotMap.INSIDE_INTAKE_MOTOR_ID);
        //TODO: Verify if limits should be put in place for motors (like 2024 version)

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        topIntake.getConfigurator().apply(config);
        bottomIntake.getConfigurator().apply(config);
        insideIntake.getConfigurator().apply(config);
    }

    // Verify if values are correct and go in the right directions

    /**
     * Pulls the game objects in the Intake.
     */
    public Command intakeIn() {
        return this.run(() -> {
            topIntake.setControl(m_output.withOutput(RobotMap.TEST_TOP_MOTOR_IN_SPEED));
            bottomIntake.setControl(m_output.withOutput(RobotMap.TEST_BOTTOM_MOTOR_IN_SPEED));
            insideIntake.setControl(m_output.withOutput(RobotMap.TEST_INSIDE_MOTOR_IN_SPEED));
            goingIn = true;
            goingOut = false;
        }).finallyDo(() -> stopMotors());
    }

    /**
     * Ejects the game objects from Intake.
     */
    public Command intakeOut() {
        return this.run(() -> {
            topIntake.setControl(m_output.withOutput(RobotMap.TEST_TOP_MOTOR_OUT_SPEED));
            bottomIntake.setControl(m_output.withOutput(RobotMap.TEST_BOTTOM_MOTOR_OUT_SPEED));
            insideIntake.setControl(m_output.withOutput(RobotMap.TEST_INSIDE_MOTOR_OUT_SPEED));
            goingIn = false;
            goingOut = true;
        }).finallyDo(() -> stopMotors());
    }
    // In: intakeMotor speed = +, Out: intakeMotor speed = -
    public void stopMotors() {
        topIntake.stopMotor();
        bottomIntake.stopMotor();
        insideIntake.stopMotor();
        goingIn = false;
        goingOut = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Going In?", goingIn);
        SmartDashboard.putBoolean("Intake Going Out?", goingOut);
    } 
}