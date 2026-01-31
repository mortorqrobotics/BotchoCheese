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

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = RobotMap.INTAKE_P_VALUE;
        slot0Configs.kI = RobotMap.INTAKE_I_VALUE;
        slot0Configs.kD = RobotMap.INTAKE_D_VALUE;
        config.Slot0 = slot0Configs;

        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0;
        currentLimits.SupplyCurrentLimitEnable = true;
        
        // Set motors to Brake mode so the climber doesn't slide down 
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        topIntake.getConfigurator().apply(config);
        bottomIntake.getConfigurator().apply(config);
        insideIntake.getConfigurator().apply(config);
    }

    // Verify if values are correct and go in the right directions

    // Pulls the game objects in the Intake.
    public Command intakeIn() {
        return this.run(() -> {
            topIntake.setControl(m_output.withOutput(RobotMap.TEST_TOP_MOTOR_IN_SPEED));
            bottomIntake.setControl(m_output.withOutput(RobotMap.TEST_BOTTOM_MOTOR_IN_SPEED));
            insideIntake.setControl(m_output.withOutput(RobotMap.TEST_INSIDE_MOTOR_IN_SPEED));
            goingIn = true;
            goingOut = false;
        }).finallyDo(() -> stopMotors());
    }

    // Ejects the game objects from Intake.
    public Command intakeOut() {
        return this.run(() -> {
            topIntake.setControl(m_output.withOutput(RobotMap.TEST_TOP_MOTOR_OUT_SPEED));
            bottomIntake.setControl(m_output.withOutput(RobotMap.TEST_BOTTOM_MOTOR_OUT_SPEED));
            insideIntake.setControl(m_output.withOutput(RobotMap.TEST_INSIDE_MOTOR_OUT_SPEED));
            goingIn = false;
            goingOut = true;
        }).finallyDo(() -> stopMotors());
    }
    
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

        SmartDashboard.putNumber("Top Intake Battery Draw", topIntake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Top Intake Motor Draw", topIntake.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Intake Battery Draw", bottomIntake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Intake Motor Draw", bottomIntake.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Inside Intake Battery Draw", insideIntake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Inside Intake Motor Draw", insideIntake.getStatorCurrent().getValueAsDouble());
    } 
}