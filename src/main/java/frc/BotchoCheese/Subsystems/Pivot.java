package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class Pivot extends SubsystemBase {
    private final TalonFX pivotLeader;
    private final TalonFX pivotFollower;

    public Pivot() {
        pivotLeader = new TalonFX(RobotMap.LEFT_PIVOT_MOTOR_ID);
        pivotFollower = new TalonFX(RobotMap.RIGHT_PIVOT_MOTOR_ID);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        Slot0Configs pivotSlot0 = new Slot0Configs();
        pivotSlot0.kS = RobotMap.PIVOT_S_VALUE;
        pivotSlot0.kV = RobotMap.PIVOT_V_VALUE;
        pivotSlot0.kA = RobotMap.PIVOT_A_VALUE;
        pivotSlot0.kP = RobotMap.PIVOT_P_VALUE;
        pivotSlot0.kI = RobotMap.PIVOT_I_VALUE;
        pivotSlot0.kD = RobotMap.PIVOT_D_VALUE;
        pivotConfig.Slot0 = pivotSlot0;

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
    }

public Command pivotUp() {
    final double upVolts = -4.0; // tune
    return this.startEnd(
        () -> pivotLeader.setVoltage(upVolts),
        () -> pivotLeader.setVoltage(0.0)
    );
}

public Command pivotDown() {
    final double downVolts = 4.0; // tune
    return this.startEnd(
        () -> pivotLeader.setVoltage(downVolts),
        () -> pivotLeader.setVoltage(0.0)
    );
}


public Command pivotUpToRotations(double deltaRotations) {
    return this.defer(() -> {
        double startPos = pivotLeader.getPosition().getValueAsDouble();
        double targetPos = startPos - Math.abs(deltaRotations); // up is negative in your setup
        final double upVolts = -4.0; // tune

        return this.startEnd(
            () -> pivotLeader.setVoltage(upVolts),
            () -> pivotLeader.setVoltage(0.0)
        ).until(() -> pivotLeader.getPosition().getValueAsDouble() <= targetPos);
    });
}




}