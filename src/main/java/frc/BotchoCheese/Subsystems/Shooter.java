package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.BotchoCheese.Constants.RobotMap;

public class Shooter extends SubsystemBase {
    // Closed-loop gains for Phoenix velocity control on the shooter wheels.
    private static final double SHOOTER_P_VALUE = 0.5;
    private static final double SHOOTER_I_VALUE = 0.0;
    private static final double SHOOTER_D_VALUE = 0.0;
    private static final double SHOOTER_S_VALUE = 0.2;
    private static final double SHOOTER_V_VALUE = 0.05;
    private static final double SHOOTER_A_VALUE = 0.01;
    private static final double SHOOTER_ACCELERATION = 160.0;
    private static final double SHOOTER_JERK = 1600.0;

    // Two back motors share one target; the right back motor follows the left back motor.
    private static final String SHOOTER_BACK_RPS_KEY = "Shooter/Back Actual RPS";
    private static final String SHOOTER_FRONT_RPS_KEY = "Shooter/Front Actual RPS";

    private final TalonFX backLeftShooter;
    private final TalonFX backRightShooter;
    private final TalonFX frontShooter;

    public Shooter() {
        backLeftShooter = new TalonFX(RobotMap.BACK_LEFT_SHOOTER_MOTOR_ID);
        backRightShooter = new TalonFX(RobotMap.BACK_RIGHT_SHOOTER_MOTOR_ID);
        frontShooter = new TalonFX(RobotMap.FRONT_SHOOTER_MOTOR_ID);
        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Velocity gains are applied to all shooter motors.

        Slot0Configs shooterSlot0 = new Slot0Configs();
        shooterSlot0.kS = SHOOTER_S_VALUE;
        shooterSlot0.kV = SHOOTER_V_VALUE;
        shooterSlot0.kA = SHOOTER_A_VALUE;
        shooterSlot0.kP = SHOOTER_P_VALUE;
        shooterSlot0.kI = SHOOTER_I_VALUE;
        shooterSlot0.kD = SHOOTER_D_VALUE;
        config.Slot0 = shooterSlot0;
        config.MotionMagic.MotionMagicAcceleration = SHOOTER_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = SHOOTER_JERK;
    
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits = currentLimits;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Preserve existing mechanism behavior while keeping positive command semantics in code.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        backLeftShooter.getConfigurator().apply(config);
        backRightShooter.getConfigurator().apply(config);
        frontShooter.getConfigurator().apply(config);

        backRightShooter.setControl(
        new Follower(backLeftShooter.getDeviceID(), MotorAlignmentValue.Aligned)
);
    }


    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

    public Command shootRps(double... rpsValues) {
        // One argument drives both back and front to the same RPS.
        // Two arguments let autos/operator code split back and front wheel targets.
        return this.runEnd(
            () -> {
                double backRps = rpsValues.length > 0 ? rpsValues[0] : 75.0;
                double frontRps = rpsValues.length > 1 ? rpsValues[1] : backRps;
                setShooterSpeeds(backRps, frontRps);
            },
            () -> {
                backLeftShooter.stopMotor();
                frontShooter.stopMotor();
            }
        );
    }

    private void setShooterSpeeds(double backShooterTargetRps, double frontShooterTargetRps) {
        // backRightShooter follows backLeftShooter, so only two velocity commands are needed here.
        backLeftShooter.setControl(shooterVelocityRequest.withVelocity(backShooterTargetRps));
        frontShooter.setControl(shooterVelocityRequest.withVelocity(frontShooterTargetRps));
    }

    @Override
    public void periodic() {
        // Phoenix velocity units are rotations per second (RPS).
        SmartDashboard.putNumber(SHOOTER_BACK_RPS_KEY, backLeftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(SHOOTER_FRONT_RPS_KEY, frontShooter.getVelocity().getValueAsDouble());
    }
}
