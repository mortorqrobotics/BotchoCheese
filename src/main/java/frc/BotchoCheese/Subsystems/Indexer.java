package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class Indexer extends SubsystemBase {
    // Motor controllers
    private final TalonFX indexer;
    private final TalonFX pivot1;
    private final TalonFX pivot2;
    //No sensor specified due to immense quantity of balls

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    //Regulates voltage (verify if works)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean isOn = false;
    
    public Indexer() {
        
        indexer = new TalonFX(RobotMap.INDEXER_MOTOR_ID);
        pivot1 = new TalonFX(RobotMap.PIVOT_1_MOTOR_ID);
        pivot2 = new TalonFX(RobotMap.PIVOT_2_MOTOR_ID);

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexer.getConfigurator().apply(config);
    }

    public Command turnIndexerOn() {
        return this.run(() -> {
            indexer.setControl(m_output.withOutput(RobotMap.INDEXER_MOTOR_SPEED));
            isOn = true;
        }).finallyDo(() -> stopMotors());
    }

    public void stopMotors() {
        indexer.stopMotor();
        pivot1.stopMotor();
        pivot2.stopMotor();
        isOn = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer On?", isOn);

        SmartDashboard.putNumber("Indexer Battery Draw", indexer.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer Motor Draw", indexer.getStatorCurrent().getValueAsDouble());
    } 
}