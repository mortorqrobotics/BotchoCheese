package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
    private final TalonFX l2Climber;
    private final TalonFX r2Climber;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    private boolean goingUp = false;
    private boolean goingDown = false;
    
    public Climber() {
        lClimber = new TalonFX(RobotMap.LEFT_CLIMBER_MOTOR_ID);
        rClimber = new TalonFX(RobotMap.RIGHT_CLIMBER_MOTOR_ID);
        l2Climber = new TalonFX(RobotMap.LEFT_CLIMBER_TWO_MOTOR_ID);
        r2Climber = new TalonFX(RobotMap.RIGHT_CLIMBER_TWO_MOTOR_ID);

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
        l2Climber.getConfigurator().apply(config);
        r2Climber.getConfigurator().apply(config);
    }

    /**
     * Commands to move the left climber. 
     */
    public Command leftClimberUp() {
        return this.run(() -> {
            lClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_UP_SPEED));
            goingUp = true;
            goingDown = false;
        }).finallyDo(() -> stopMotors());
    }

    public Command leftClimberDown() {
        return this.run(() -> {
            lClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_DOWN_SPEED));
            goingUp = false;
            goingDown = true;
        }).finallyDo(() -> stopMotors());
    }
    public Command left2ClimberUp() {
        return this.run(() -> {
            l2Climber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_TWO_UP_SPEED));
            goingUp = true;
            goingDown = false;
        }).finallyDo(() -> stopMotors());
    }
    public Command left2ClimberDown() {
        return this.run(() -> {
            l2Climber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_LEFT_TWO_DOWN_SPEED));
            goingUp = false;
            goingDown = true;
        }).finallyDo(() -> stopMotors());
    }
    
    /**
     * Commands to move the right climber.
     */
    public Command rightClimberUp() {
        return this.run(() -> {
            rClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_UP_SPEED));
            goingUp = true;
            goingDown = false;
        }).finallyDo(() -> stopMotors());
    }

    public Command rightClimberDown() {
        return this.run(() -> {
            rClimber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_DOWN_SPEED));
            goingUp = false;
            goingDown = true;
        }).finallyDo(() -> stopMotors());
    }
     public Command right2ClimberUp() {
        return this.run(() -> {
            r2Climber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_TWO_UP_SPEED));
            goingUp = true;
            goingDown = false;
        }).finallyDo(() -> stopMotors());
    }
    public Command right2ClimberDown() {
        return this.run(() -> {
            r2Climber.setControl(m_output.withOutput(RobotMap.TEST_MOTOR_RIGHT_TWO_DOWN_SPEED));
            goingUp = false;
            goingDown = true;
        }).finallyDo(() -> stopMotors());
    }

    public void stopMotors() {
        lClimber.stopMotor();
        l2Climber.stopMotor();
        rClimber.stopMotor();
        rClimber.stopMotor();
        goingUp = false;
        goingDown = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber Going Up?", goingUp);
        SmartDashboard.putBoolean("Climber Going Down?", goingDown);
    } 
}