package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final DutyCycleOut intakeDuty = new DutyCycleOut(0);

    public Intake() {
        intakeMotor = new TalonFX(RobotMap.INTAKE_MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        // Starting point for X44 current limits; tune on robot once hardware is installed.
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotor.getConfigurator().apply(config);
    }

    public Command runIntake(double percent) {
        // Runs until the command ends, then stops the intake motor.
        return this.startEnd(
            () -> intakeMotor.setControl(intakeDuty.withOutput(clamp(percent))),
            () -> intakeMotor.setControl(intakeDuty.withOutput(0.0))
        );
    }

    private static double clamp(double percent) {
        return Math.max(-1.0, Math.min(1.0, percent));
    }
}
