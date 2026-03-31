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

public class Intake extends SubsystemBase {
    private final TalonFXS intakeMotor;

    // Control requests
    private final DutyCycleOut intakeDuty = new DutyCycleOut(0);

    
    public Intake() {
        
        intakeMotor = new TalonFXS(RobotMap.INTAKE_MOTOR_ID);
        // --- INTAKE CONFIGURATION (Minion) ---
        TalonFXSConfiguration intakeConfig = new TalonFXSConfiguration();
        
        CurrentLimitsConfigs intakeLimits = new CurrentLimitsConfigs();
        intakeLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        intakeLimits.StatorCurrentLimitEnable = true;
        intakeLimits.SupplyCurrentLimit = 30.0;
        intakeLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits = intakeLimits;
    
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        intakeMotor.getConfigurator().apply(intakeConfig);
    }


public Command runIntake(double percent) {
    return this.startEnd(
        () -> intakeMotor.setControl(
            intakeDuty.withOutput(-Math.max(-1.0, Math.min(1.0, percent)))
        ),
        () -> intakeMotor.setControl(intakeDuty.withOutput(0.0))
    );
}



}
