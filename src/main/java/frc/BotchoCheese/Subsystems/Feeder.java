package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;


public class Feeder extends SubsystemBase {
    // Single Minion motor controlled by a Talon FXS
    private final TalonFX feeder;

    // Control request object
    private final DutyCycleOut feederDuty  = new DutyCycleOut(0);

    public Feeder() {
        // Updated to TalonFXS class
        feeder = new TalonFX(RobotMap.FEEDER_MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        feeder.getConfigurator().apply(config);
    }

    public Command runFeeder() {
    return this.startEnd(
        () -> feeder.setControl(feederDuty.withOutput(RobotMap.FEEDER_SPEED)),
        () -> feeder.stopMotor()
    );
}

public Command reverseFeeder() {
    return this.startEnd(
        () -> feeder.setControl(feederDuty.withOutput(-RobotMap.FEEDER_SPEED)),
        () -> feeder.stopMotor()
    );
}

}
