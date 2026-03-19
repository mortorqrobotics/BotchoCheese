package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class Climber extends SubsystemBase {
    // Motor controllers
    private final TalonFX lClimber;
    private final TalonFX rClimber;

    // Limit switches
    private final CANcoder encoder;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean goingUp = true;
    
    public Climber() {
        lClimber = new TalonFX(RobotMap.LEFT_CLIMBER_MOTOR_ID);
        rClimber = new TalonFX(RobotMap.RIGHT_CLIMBER_MOTOR_ID);
        encoder = new CANcoder(RobotMap.ENCODER_ID);

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = RobotMap.CLIMBER_P_VALUE;
        slot0Configs.kI = RobotMap.CLIMBER_I_VALUE;
        slot0Configs.kD = RobotMap.CLIMBER_D_VALUE;
        config.Slot0 = slot0Configs;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        lClimber.getConfigurator().apply(config);
        rClimber.getConfigurator().apply(config);
    }

    // public Boolean atLimit(){
    //     double position = encoder.getAbsolutePosition().getValueAsDouble();
    //     // if(encoder.getAbsolutePosition().getValueAsDouble() < RobotMap.CLIMBER_EXTENSION_LIMIT 
    //     // || encoder.getAbsolutePosition().getValueAsDouble() > 0){
    //     //     return true;
    //     // }
    //     return position >= RobotMap.CLIMBER_EXTENSION_LIMIT || position <= 0.0;
    // }
    public boolean atUpperLimit() {
        return encoder.getAbsolutePosition().getValueAsDouble() >= RobotMap.CLIMBER_EXTENSION_LIMIT;
    }

    public boolean atLowerLimit() {
        return encoder.getAbsolutePosition().getValueAsDouble() <= 0.0;
    }

    // Commands to move the climber.
    public Command manualClimberUp() {
        return this.run(() -> {
            if(!atUpperLimit()) {
                lClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_UP_SPEED));
                rClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_UP_SPEED));
                goingUp = true;
            }
        }).finallyDo(() -> stopMotors());
    }

    public Command manualClimberDown() {
        return this.run(() -> {
            if(!atLowerLimit()) {
                lClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_UP_SPEED));
                rClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_DOWN_SPEED));
                goingUp = false;
            }
        }).finallyDo(() -> stopMotors());
    }

    // public Command manualClimber(boolean direction) {
    //     if (direction == true) {
    //         return this.run(() -> {
    //             if(!atUpperLimit()) {
    //                 lClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_UP_SPEED));
    //                 rClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_UP_SPEED));
    //                 goingUp = true;
    //             }
    //         }).finallyDo(() -> stopMotors());
    //     }
    //     else {
    //         return this.run(() -> {
    //             if(!atLowerLimit()) {
    //                 lClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_DOWN_SPEED));
    //                 rClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_DOWN_SPEED));
    //                 goingUp = false;
    //             }
    //         }).finallyDo(() -> stopMotors());
    //     }
    // }

    public Command automaticClimberUp(){
        return this.run(() -> {
            lClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_UP_SPEED));
            rClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_UP_SPEED));
            goingUp = false;
        }).until(this::atUpperLimit).finallyDo(() -> stopMotors());
    }

    public Command automaticClimberDown(){
        return this.run(() -> {
            lClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_DOWN_SPEED));
            rClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_DOWN_SPEED));
            goingUp = false;
        }).until(this::atLowerLimit).finallyDo(() -> stopMotors());
    }
    
    public void stopMotors() {
        lClimber.stopMotor();
        rClimber.stopMotor();
        goingUp = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber Going Up?", goingUp);
        SmartDashboard.putNumber("Climber Position", lClimber.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Left Climber 1 Battery Draw", lClimber.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Climber 1 Motor Draw", rClimber.getStatorCurrent().getValueAsDouble());
    }
}