package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class Intake extends SubsystemBase {
    private static final double PIVOT_S_VALUE = 9.0;
    private static final double PIVOT_V_VALUE = 0.0;
    private static final double PIVOT_A_VALUE = 0.0;
    private static final double PIVOT_P_VALUE = 0.0;
    private static final double PIVOT_I_VALUE = 0.0;
    private static final double PIVOT_D_VALUE = 0.0;
    private static final double PIVOT_CRUISE_VELOCITY = 200.0;
    private static final double PIVOT_ACCELERATION = 20.0;
    private static final double PIVOT_JERK = 20.0;

    // Hardware: 2x Kraken X44s for Pivot, 1x Minion for Intake

    private final TalonFX pivotLeader;
    private final TalonFX pivotFollower;
    private final TalonFXS intakeMotor;

    // Control requests
    private final DutyCycleOut intakeDuty = new DutyCycleOut(0);

    
    public Intake() {
        pivotLeader = new TalonFX(RobotMap.LEFT_PIVOT_MOTOR_ID);
        pivotFollower = new TalonFX(RobotMap.RIGHT_PIVOT_MOTOR_ID);
        intakeMotor = new TalonFXS(RobotMap.INTAKE_MOTOR_ID);

        var pivotConfig = new TalonFXConfiguration();
        
        // PID configuration for moving the pivot to set positions
        Slot0Configs pivotSlot0 = new Slot0Configs();
        pivotSlot0.kS = PIVOT_S_VALUE;
        pivotSlot0.kV = PIVOT_V_VALUE;
        pivotSlot0.kA = PIVOT_A_VALUE;
        pivotSlot0.kP = PIVOT_P_VALUE;
        pivotSlot0.kI = PIVOT_I_VALUE;
        pivotSlot0.kD = PIVOT_D_VALUE;
        pivotConfig.Slot0 = pivotSlot0;

        // var motionMagicConfigs = pivotConfig.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = PIVOT_CRUISE_VELOCITY;
        // motionMagicConfigs.MotionMagicAcceleration = PIVOT_ACCELERATION;
        // motionMagicConfigs.MotionMagicJerk = PIVOT_JERK;

        // Current limits to protect the X44s and the pivot mechanism
        CurrentLimitsConfigs pivotLimits = new CurrentLimitsConfigs();
        pivotLimits.StatorCurrentLimit = 60.0; 
        pivotLimits.StatorCurrentLimitEnable = true;
        pivotLimits.SupplyCurrentLimit = 40.0;
        pivotLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits = pivotLimits;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotLeader.getConfigurator().apply(pivotConfig);
        pivotFollower.getConfigurator().apply(pivotConfig);

        pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), MotorAlignmentValue.Opposed));

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
