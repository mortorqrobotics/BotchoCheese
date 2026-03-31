package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class Pivot extends SubsystemBase {
    private static final double PIVOT_S_VALUE = 9.0;
    private static final double PIVOT_V_VALUE = 0.0;
    private static final double PIVOT_A_VALUE = 0.0;
    private static final double PIVOT_P_VALUE = 0.0;
    private static final double PIVOT_I_VALUE = 0.0;
    private static final double PIVOT_D_VALUE = 0.0;
    private static final double PIVOT_CRUISE_VELOCITY = 200.0;
    private static final double PIVOT_ACCELERATION = 20.0;
    private static final double PIVOT_JERK = 20.0;
    //private static final double PIVOT_TARGET_ROTATIONS = 8.0;
    //private static final double PIVOT_FEEDFORWARD = 9.0;

    private final TalonFX pivotLeader;
    private final TalonFX pivotFollower;

    public Pivot() {
        pivotLeader = new TalonFX(RobotMap.LEFT_PIVOT_MOTOR_ID);
        pivotFollower = new TalonFX(RobotMap.RIGHT_PIVOT_MOTOR_ID);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        Slot0Configs pivotSlot0 = new Slot0Configs();
        pivotSlot0.kS = PIVOT_S_VALUE;
        pivotSlot0.kV = PIVOT_V_VALUE;
        pivotSlot0.kA = PIVOT_A_VALUE;
        pivotSlot0.kP = PIVOT_P_VALUE;
        pivotSlot0.kI = PIVOT_I_VALUE;
        pivotSlot0.kD = PIVOT_D_VALUE;
        pivotConfig.Slot0 = pivotSlot0;

        // Keep the motion profile tuning here with the rest of the pivot config.
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = PIVOT_CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = PIVOT_ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicJerk = PIVOT_JERK;

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

    public Command pivotUp() {
        final double upVolts = -4.0;
        return this.startEnd(
            () -> pivotLeader.setVoltage(upVolts),
            () -> pivotLeader.setVoltage(0.0)
        );
    }

    public Command pivotDown() {
        final double downVolts = 4.0;
        return this.startEnd(
            () -> pivotLeader.setVoltage(downVolts),
            () -> pivotLeader.setVoltage(0.0)
        );
    }

    public Command pivotUpToRotations(double deltaRotations) {
        return this.defer(() -> {
            double startPos = pivotLeader.getPosition().getValueAsDouble();
            double targetPos = startPos - Math.abs(deltaRotations);
            final double upVolts = -4.0;

            return this.startEnd(
                () -> pivotLeader.setVoltage(upVolts),
                () -> pivotLeader.setVoltage(0.0)
            ).until(() -> pivotLeader.getPosition().getValueAsDouble() <= targetPos);
        });
    }
}
