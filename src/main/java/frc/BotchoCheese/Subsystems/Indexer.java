package frc.BotchoCheese.Subsystems;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class Indexer extends SubsystemBase {
    // Motor controllers
    private final TalonFXS indexer;
    //No sensor specified due to immense quantity of balls

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    //Regulates voltage (verify if works)
    private final DutyCycleOut indexerDuty = new DutyCycleOut(0);

    
    public Indexer() {
        
        indexer = new TalonFXS(RobotMap.INDEXER_MOTOR_ID);

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        
        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexer.getConfigurator().apply(config);
    }

    public Command runIndexer(double percent) {
        return this.startEnd(
            () -> indexer.setControl(indexerDuty.withOutput(Math.max(-1.0, Math.min(1.0, percent)))),
            () -> indexer.setControl(indexerDuty.withOutput(0.0))
        );
    }

}
