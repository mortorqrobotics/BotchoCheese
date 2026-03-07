package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class Intake extends SubsystemBase {
    // Hardware: 2x Kraken X44s for Pivot, 1x Minion for Intake
    private final TalonFX pivotLeader;
    private final TalonFX pivotFollower;
    private final TalonFX intakeMotor;
    private final CoreCANrange CANrange;

    // Control requests
    private final DutyCycleOut intakeOutput = new DutyCycleOut(0);
    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0);

    // Telemetry state variables
    private boolean goingIn = false;
    private boolean pivotUp = true;
    
    public Intake() {
        // Initialize motors (You will need to add these new IDs to your RobotMap)
        pivotLeader = new TalonFX(RobotMap.PIVOT_1_MOTOR_ID);
        pivotFollower = new TalonFX(RobotMap.PIVOT_2_MOTOR_ID);
        intakeMotor = new TalonFX(RobotMap.INTAKE_MOTOR_ID);
        CANrange = new CoreCANrange(RobotMap.CAN_RANGE_ID);

        // --- PIVOT CONFIGURATION (Kraken X44s) ---
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        
        // PID configuration for moving the pivot to set positions
        Slot0Configs pivotSlot0 = new Slot0Configs();
        pivotSlot0.kS = RobotMap.PIVOT_S_VALUE;
        pivotSlot0.kV = RobotMap.PIVOT_V_VALUE;
        pivotSlot0.kA = RobotMap.PIVOT_A_VALUE;
        pivotSlot0.kP = RobotMap.PIVOT_P_VALUE;
        pivotSlot0.kI = RobotMap.PIVOT_I_VALUE;
        pivotSlot0.kD = RobotMap.PIVOT_D_VALUE;
        pivotConfig.Slot0 = pivotSlot0;

        MotionMagicConfigs motionMagicConfigs = pivotConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = RobotMap.INTAKE_CRUISE_VELOCITY; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = RobotMap.INTAKE_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = RobotMap.INTAKE_JERK; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // Current limits to protect the X44s and the pivot mechanism
        CurrentLimitsConfigs pivotLimits = new CurrentLimitsConfigs();
        pivotLimits.StatorCurrentLimit = 60.0; 
        pivotLimits.StatorCurrentLimitEnable = true;
        pivotLimits.SupplyCurrentLimit = 40.0;
        pivotLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits = pivotLimits;

        // Pivot MUST be in brake mode to hold its position against gravity
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotLeader.getConfigurator().apply(pivotConfig);
        pivotFollower.getConfigurator().apply(pivotConfig);

        // Set the second X44 to strictly follow the leader. 
        // Note: Change 'false' to 'true' if the follower motor needs to be inverted relative to the leader!
        pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), MotorAlignmentValue.Aligned));

        // --- INTAKE CONFIGURATION (Minion) ---
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        
        CurrentLimitsConfigs intakeLimits = new CurrentLimitsConfigs();
        intakeLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        intakeLimits.StatorCurrentLimitEnable = true;
        intakeLimits.SupplyCurrentLimit = 30.0;
        intakeLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits = intakeLimits;
        
        // Typically intakes run in Coast mode so objects don't get stuck, 
        // but keeping it Brake if your game piece requires firm holding.
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeMotor.getConfigurator().apply(intakeConfig);
    }

    // --- PIVOT METHODS ---

    /**
     * Moves the pivot to a target position (in rotations).
     */
    public Command setPivotPosition(double targetRotations) {
        return this.run(() -> {
            pivotLeader.setControl(pivotPositionRequest.withPosition(targetRotations));
        });
    }

    public Command setPivotUp() {
        return this.run(() -> {
            pivotLeader.setControl(pivotPositionRequest.withPosition(RobotMap.PIVOT_UP_POSITION));
            pivotUp = true;
        });
    }

    public Command setPivotDown() {
        return this.run(() -> {
            pivotLeader.setControl(pivotPositionRequest.withPosition(RobotMap.PIVOT_DOWN_POSITION));
            pivotUp = false;
        });
    }

    // --- INTAKE METHODS ---

    // Pulls the game objects into the Intake using the single Minion.
    public Command startIntake() {
        return this.run(() -> {
            intakeMotor.setControl(intakeOutput.withOutput(RobotMap.INTAKE_SPEED));
            goingIn = true;
        }).finallyDo(() -> stopIntake());
    }

    // Ejects the game objects from the Intake.
    public Command intakeOut() {
        return this.run(() -> {
            intakeMotor.setControl(intakeOutput.withOutput(-RobotMap.INTAKE_SPEED));
            goingIn = false;
        }).finallyDo(() -> stopIntake());
    }

    // Checks whether the intake is full or not.
    // TODO Refine the command later
    public Command isIndexFull() {
        return this.run(() -> {
            while(!pivotUp) {
                while(goingIn) {
                    if(CANrange.getDistance().getValueAsDouble() < RobotMap.CAN_RANGE_DISTANCE_THRESHOLD) {
                        stopIntake();
                    }
                    else {
                        startIntake();
                    }
                }
            }
        });
    }
    
    public void stopIntake() {
        intakeMotor.stopMotor();
        goingIn = false;
    }

    // Completely stop everything (Useful for an emergency stop or disable command)
    public void stopPivot() {
        pivotLeader.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Going In?", goingIn);

        // Intake Telemetry
        SmartDashboard.putNumber("Intake Battery Draw", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Draw", intakeMotor.getStatorCurrent().getValueAsDouble());
        
        // Pivot Telemetry
        SmartDashboard.putNumber("Pivot Position (Rot)", pivotLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Leader Draw", pivotLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Follower Draw", pivotFollower.getStatorCurrent().getValueAsDouble());
    } 
}