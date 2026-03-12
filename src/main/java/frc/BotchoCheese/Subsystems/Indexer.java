package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class Indexer extends SubsystemBase {
    // Motor controllers
    private final TalonFXS indexer;
    //No sensor specified due to immense quantity of balls

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    //Regulates voltage (verify if works)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean isOn = false;
    
    public Indexer() {
        
        indexer = new TalonFXS(RobotMap.INDEXER_MOTOR_ID);

        // Apply basic configuration
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        
        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexer.getConfigurator().apply(config);

        System.out.println("Indexer constructed=====================");
    }

    public Command indexerOn() {
        System.out.println("indexerOn executed=====================");
        return this.run(() -> {
            indexer.setControl(m_output.withOutput(RobotMap.INDEXER_MOTOR_SPEED));
            isOn = true;
        }).finallyDo(() -> stopMotors());
    }

    public Command reverseIndexer() {
        System.out.println("reverseIndexer executed=====================");
        return this.run(() -> {
            indexer.setControl(m_output.withOutput(-RobotMap.INDEXER_MOTOR_SPEED));
            isOn = true;
        }).finallyDo(() -> stopMotors());
    }

    public void stopMotors() {
        indexer.stopMotor();
        isOn = false;
        System.out.println("indexer stopMotors executed=====================");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer On?", isOn);

        SmartDashboard.putNumber("Indexer Battery Draw", indexer.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer Motor Draw", indexer.getStatorCurrent().getValueAsDouble());
    } 
}