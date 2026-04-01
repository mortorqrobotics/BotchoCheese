package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class Pivot extends SubsystemBase {
    private static final double PIVOT_VOLTS = 4.0;
    //private static final double PIVOT_TARGET_ROTATIONS = 8.0;
    //private static final double PIVOT_FEEDFORWARD = 9.0;

    private final TalonFX pivotLeader;
    private final TalonFX pivotFollower;

    public Pivot() {
        pivotLeader = new TalonFX(RobotMap.LEFT_PIVOT_MOTOR_ID);
        pivotFollower = new TalonFX(RobotMap.RIGHT_PIVOT_MOTOR_ID);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

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

    public void enableBrakeMode() {
        pivotLeader.setNeutralMode(NeutralModeValue.Brake);
        pivotFollower.setNeutralMode(NeutralModeValue.Brake);
    }

    public void disableBrakeMode() {
        pivotLeader.setNeutralMode(NeutralModeValue.Coast);
        pivotFollower.setNeutralMode(NeutralModeValue.Coast);
    }

    public void zeroPivotEncoder() {
        pivotLeader.setPosition(0.0);
        pivotFollower.setPosition(0.0);
    }

    public double getPivotRotations() {
        return pivotLeader.getPosition().getValueAsDouble();
    }

    public Command pivotUp() {
        return this.startEnd(
            () -> pivotLeader.setVoltage(-PIVOT_VOLTS),
            () -> pivotLeader.setVoltage(0.0)
        );
    }

    public Command pivotDown() {
        return this.startEnd(
            () -> pivotLeader.setVoltage(PIVOT_VOLTS),
            () -> pivotLeader.setVoltage(0.0)
        );
    }

    public Command pivotUpToRotations(double deltaRotations) {
        return this.defer(() -> {
            double startPos = pivotLeader.getPosition().getValueAsDouble();
            double targetPos = startPos - Math.abs(deltaRotations);

            return this.startEnd(
                () -> pivotLeader.setVoltage(-PIVOT_VOLTS),
                () -> pivotLeader.setVoltage(0.0)
            ).until(() -> pivotLeader.getPosition().getValueAsDouble() <= targetPos);
        });
    }
}
