package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Constants.RobotMap;

public class Shooter extends SubsystemBase {
    private static final String SHOOTER_ACTUAL_RPS_KEY = "Shooter Actual RPS";
    private static final String BACK_LEFT_SHOOTER_ACTUAL_RPS_KEY = "Shooter Back Left Actual RPS";
    private static final String FRONT_SHOOTER_ACTUAL_RPS_KEY = "Shooter Front Actual RPS";
    private static final double DEFAULT_LOFT_FRONT_SPEED_SCALE = 0.75;
    private static final double SHOOTER_P_VALUE = 0.5;
    private static final double SHOOTER_I_VALUE = 0.0;
    private static final double SHOOTER_D_VALUE = 0.0;
    private static final double SHOOTER_S_VALUE = 0.2;
    private static final double SHOOTER_V_VALUE = 0.05;
    private static final double SHOOTER_A_VALUE = 0.01;
    private static final double SHOOTER_ACCELERATION = 160.0;
    private static final double SHOOTER_JERK = 1600.0;

    // Motor controllers
    private final TalonFX backLeftShooter;
    private final TalonFX backRightShooter;
    private final TalonFX frontShooter;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)


    public Shooter() {
        backLeftShooter = new TalonFX(RobotMap.BACK_LEFT_SHOOTER_MOTOR_ID);
        backRightShooter = new TalonFX(RobotMap.BACK_RIGHT_SHOOTER_MOTOR_ID);
        frontShooter = new TalonFX(RobotMap.FRONT_SHOOTER_MOTOR_ID);
        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID Shooter Values

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

        backLeftShooter.getConfigurator().apply(config);
        backRightShooter.getConfigurator().apply(config);
        frontShooter.getConfigurator().apply(config);

        backLeftShooter.setNeutralMode(NeutralModeValue.Brake);
        backRightShooter.setNeutralMode(NeutralModeValue.Brake);
        frontShooter.setNeutralMode(NeutralModeValue.Brake);

        backRightShooter.setControl(
        new Follower(backLeftShooter.getDeviceID(), MotorAlignmentValue.Aligned)
);
    }


    // Command to shoot at a specific RPS (revolutions per second)
private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

public Command shootRps(double targetRps) {
    return this.startEnd(
        () -> {
            backLeftShooter.setControl(shooterVelocityRequest.withVelocity(-targetRps));
            frontShooter.setControl(shooterVelocityRequest.withVelocity(-targetRps));
        },
        () -> {
            backLeftShooter.stopMotor();
            frontShooter.stopMotor();
        }
    );
}

public Command shootRps(DoubleSupplier targetRpsSupplier) {
    return this.runEnd(
        () -> {
            double targetRps = targetRpsSupplier.getAsDouble();
            backLeftShooter.setControl(shooterVelocityRequest.withVelocity(-targetRps));
            frontShooter.setControl(shooterVelocityRequest.withVelocity(-targetRps));
        },
        () -> {
            backLeftShooter.stopMotor();
            frontShooter.stopMotor();
        }
    );
}

public Command shootLoftRps(double backShooterTargetRps) {
    return shootLoftRps(backShooterTargetRps, DEFAULT_LOFT_FRONT_SPEED_SCALE);
}

public Command shootLoftRps(double backShooterTargetRps, double frontSpeedScale) {
    return this.startEnd(
        () -> setLoftShotSpeeds(backShooterTargetRps, frontSpeedScale),
        () -> {
            backLeftShooter.stopMotor();
            frontShooter.stopMotor();
        }
    );
}

public Command shootLoftRps(DoubleSupplier backShooterTargetRpsSupplier, double frontSpeedScale) {
    return this.runEnd(
        () -> setLoftShotSpeeds(backShooterTargetRpsSupplier.getAsDouble(), frontSpeedScale),
        () -> {
            backLeftShooter.stopMotor();
            frontShooter.stopMotor();
        }
    );
}

public Command shootLoftRps(
    DoubleSupplier backShooterTargetRpsSupplier,
    DoubleSupplier frontSpeedScaleSupplier
) {
    return this.runEnd(
        () -> setLoftShotSpeeds(
            backShooterTargetRpsSupplier.getAsDouble(),
            frontSpeedScaleSupplier.getAsDouble()
        ),
        () -> {
            backLeftShooter.stopMotor();
            frontShooter.stopMotor();
        }
    );
}

public Command frontShooterOutRps(double targetRps) {
    return this.startEnd(
        () -> frontShooter.setControl(shooterVelocityRequest.withVelocity(Math.abs(targetRps))),
        () -> frontShooter.stopMotor()
    );
}

public double getBackLeftRps() {
    return Math.abs(backLeftShooter.getVelocity().getValueAsDouble());
}

public double getFrontRps() {
    return Math.abs(frontShooter.getVelocity().getValueAsDouble());
}

public double getAverageRps() {
    return (getBackLeftRps() + getFrontRps()) / 2.0;
}

private void setLoftShotSpeeds(double backShooterTargetRps, double frontSpeedScale) {
    double frontShooterTargetRps = backShooterTargetRps * Math.abs(frontSpeedScale);
    backLeftShooter.setControl(shooterVelocityRequest.withVelocity(-backShooterTargetRps));
    frontShooter.setControl(shooterVelocityRequest.withVelocity(-frontShooterTargetRps));
}

@Override
public void periodic() {
    SmartDashboard.putNumber(SHOOTER_ACTUAL_RPS_KEY, getAverageRps());
    SmartDashboard.putNumber(BACK_LEFT_SHOOTER_ACTUAL_RPS_KEY, getBackLeftRps());
    SmartDashboard.putNumber(FRONT_SHOOTER_ACTUAL_RPS_KEY, getFrontRps());
}
}
