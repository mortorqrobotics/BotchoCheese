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


// import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Constants.RobotMap;

public class Shooter extends SubsystemBase {
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
        shooterSlot0.kS = RobotMap.SHOOTER_S_VALUE;
        shooterSlot0.kV = RobotMap.SHOOTER_V_VALUE;
        shooterSlot0.kA = RobotMap.SHOOTER_A_VALUE;
        shooterSlot0.kP = RobotMap.SHOOTER_P_VALUE;
        shooterSlot0.kI = RobotMap.SHOOTER_I_VALUE;
        shooterSlot0.kD = RobotMap.SHOOTER_D_VALUE;
        config.Slot0 = shooterSlot0;
    
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
}
