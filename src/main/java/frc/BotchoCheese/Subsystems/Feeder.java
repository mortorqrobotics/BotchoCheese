package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;


public class Feeder extends SubsystemBase {
    // Single Minion motor controlled by a Talon FXS
    private final TalonFX feederLMotor;
    private final TalonFX feederRMotor;

    // Control request object
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean isFeeding = false;

    
    public Feeder() {
        // Updated to TalonFXS class
        feederLMotor = new TalonFX(RobotMap.FEEDER_MOTOR_L_ID);
        feederRMotor = new TalonFX(RobotMap.FEEDER_MOTOR_R_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // /** * CRITICAL: Set the Motor Arrangement. 
        //  * This tells the FXS it is connected to a Minion via the JST sensor port.
        //  */
        // config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // PID Values for velocity control
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = RobotMap.FEEDER_P_VALUE;
        slot0Configs.kI = RobotMap.FEEDER_I_VALUE;
        slot0Configs.kD = RobotMap.FEEDER_D_VALUE;
        config.Slot0 = slot0Configs;

        // Motion Magic is disabled for the feeder while using basic percent output control.
        // MotionMagicConfigs feederMotionMagicConfigs = config.MotionMagic;
        // feederMotionMagicConfigs.MotionMagicAcceleration = RobotMap.FEEDER_ACCELERATION;
        // feederMotionMagicConfigs.MotionMagicJerk = RobotMap.FEEDER_JERK;
        // feederMotionMagicConfigs.MotionMagicCruiseVelocity = RobotMap.FEEDER_CRUISE_VELOCITY;
        
        // Current Limits optimized for a Minion
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Protection against stalls
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;
        
        // Use Brake mode to ensure the note stops exactly when we want
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply the configuration to the FXS
        feederLMotor.getConfigurator().apply(config);
        feederRMotor.getConfigurator().apply(config);

        feederRMotor.setControl(new Follower(feederLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Runs the feeder at the defined constant speed.
     */
    public Command runFeeder() {
        return this.startEnd(
            () -> {
                feederLMotor.setControl(m_output.withOutput(RobotMap.FEEDER_SPEED));
                isFeeding = true;
            },
            () -> {
                stopMotor();
            }
        );
    }

    /**
     * Reverses the feeder to clear a jam or outtake.
     */
    public Command reverseFeeder() {
        return this.run(() -> {
            feederLMotor.setControl(m_output.withOutput(-RobotMap.FEEDER_SPEED*0.35));
        }).finallyDo((interrupted) -> stopMotor());
    }

    public void stopMotor() {
        feederLMotor.stopMotor();
        isFeeding = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Feeder Running?", isFeeding);
        // Using getValueAsDouble() to keep it simple for your dashboard
        SmartDashboard.putNumber("Feeder L Current", feederLMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder R Current", feederRMotor.getStatorCurrent().getValueAsDouble());
    } 
}
