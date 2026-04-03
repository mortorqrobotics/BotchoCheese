package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;


public class Feeder extends SubsystemBase {
    // Single motor that feeds notes from the conveyor path into the shooter.
    private final TalonFX feeder;

    // Phoenix request reused for percent-output commands.
    private final DutyCycleOut feederDuty  = new DutyCycleOut(0);

    public Feeder() {
        feeder = new TalonFX(RobotMap.FEEDER_MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        feeder.getConfigurator().apply(config);
    }


    public Command runFeeder(double percent) {
        // Runs until the command ends, then stops the feeder motor.
        return this.startEnd(
            () -> feeder.setControl(feederDuty.withOutput(Math.max(-1.0, Math.min(1.0, percent)))),
            () -> feeder.setControl(feederDuty.withOutput(0.0))
        );
    }

}
