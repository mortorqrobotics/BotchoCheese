package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class Indexer extends SubsystemBase {
    private final TalonFXS indexer;
    private final DutyCycleOut indexerDuty = new DutyCycleOut(0);

    public Indexer() {
        indexer = new TalonFXS(RobotMap.INDEXER_MOTOR_ID);

        TalonFXSConfiguration config = new TalonFXSConfiguration();

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        indexer.getConfigurator().apply(config);
    }

    public Command runIndexer(double percent) {
        return this.startEnd(
            () -> indexer.setControl(indexerDuty.withOutput(Math.max(-1.0, Math.min(1.0, percent)))),
            () -> indexer.setControl(indexerDuty.withOutput(0.0))
        );
    }
}
