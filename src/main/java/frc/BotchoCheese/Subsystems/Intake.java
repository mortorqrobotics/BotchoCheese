package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class Intake extends SubsystemBase {
    // Hardware: 2x Kraken X44s for Pivot, 1x Minion for Intake
    private final TalonFX pivotLeader;
    private final TalonFX pivotFollower;
    private final TalonFXS intakeMotor;
    // private final CoreCANrange CANrange;

    // Control requests
    private final DutyCycleOut intakeOutput = new DutyCycleOut(0);
    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut pivot_output = new DutyCycleOut(0);

    // Telemetry state variables
    private boolean goingIn = false;
    private boolean pivotUp = true;
    
    public Intake() {
        // Initialize motors (You will need to add these new IDs to your RobotMap)
        pivotLeader = new TalonFX(RobotMap.LEFT_PIVOT_MOTOR_ID);
        pivotFollower = new TalonFX(RobotMap.RIGHT_PIVOT_MOTOR_ID);
        intakeMotor = new TalonFXS(RobotMap.INTAKE_MOTOR_ID);
        // CANrange = new CoreCANrange(RobotMap.CAN_RANGE_ID);

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
        motionMagicConfigs.MotionMagicCruiseVelocity = RobotMap.PIVOT_CRUISE_VELOCITY; // Target cruise velocity of 20 rps
        motionMagicConfigs.MotionMagicAcceleration = RobotMap.PIVOT_ACCELERATION; // Target acceleration of 160 rps^2 (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = RobotMap.PIVOT_JERK; // Target jerk of 1600 rps^3 (0.1 seconds)

        // Current limits to protect the X44s and the pivot mechanism
        CurrentLimitsConfigs pivotLimits = new CurrentLimitsConfigs();
        pivotLimits.StatorCurrentLimit = 60.0; 
        pivotLimits.StatorCurrentLimitEnable = true;
        pivotLimits.SupplyCurrentLimit = 40.0;
        pivotLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits = pivotLimits;

        // Pivot MUST be in brake mode to hold its position against gravity
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Commutation/MotorArrangement is not available on TalonFXConfiguration in this CTRE Phoenix version,
        // so we do not set pivotConfig.Commutation here.

        pivotLeader.getConfigurator().apply(pivotConfig);
        pivotFollower.getConfigurator().apply(pivotConfig);

        // Set the second X44 to strictly follow the leader. 
        // Note: Change 'false' to 'true' if the follower motor needs to be inverted relative to the leader!
        pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        // --- INTAKE CONFIGURATION (Minion) ---
        TalonFXSConfiguration intakeConfig = new TalonFXSConfiguration();
        
        CurrentLimitsConfigs intakeLimits = new CurrentLimitsConfigs();
        intakeLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        intakeLimits.StatorCurrentLimitEnable = true;
        intakeLimits.SupplyCurrentLimit = 30.0;
        intakeLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits = intakeLimits;
        
        // Typically intakes run in Coast mode so objects don't get stuck, 
        // but keeping it Brake if your game piece requires firm holding.
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        //TODO pivot minion config 

        intakeMotor.getConfigurator().apply(intakeConfig);

        System.out.println("Intake constructed======================");
    }

    // --- PIVOT METHODS ---

    /**
     * Moves the pivot to a target position (in rotations).
     */
    public Command setPivotPosition(double targetRotations) {
        System.out.println("setPivotPosition executed=====================");
        return this.run(() -> {
            pivotLeader.setControl(pivotPositionRequest.withPosition(targetRotations));
        });
    }

    public Command setPivotUp() {
        System.out.println("setPivotUp executed=====================");
        return this.run(() -> {
            if (isPivotStalling()) {
                handlePivotHardStop(RobotMap.PIVOT_UP_POSITION, true);
                return;
            }

            pivotLeader.setControl(pivotPositionRequest.withPosition(RobotMap.PIVOT_UP_POSITION));
            pivotUp = true;
        });
    }

    public Command setPivotDown() {
        System.out.println("setPivotDown executed=====================");
        return this.run(() -> {
            if (isPivotStalling()) {
                handlePivotHardStop(RobotMap.PIVOT_DOWN_POSITION, false);
                return;
            }

            pivotLeader.setControl(pivotPositionRequest.withPosition(RobotMap.PIVOT_DOWN_POSITION));
            pivotUp = false;
        });
    }

    public Command pivotUp() {
        return this.run(() -> {
                        pivotLeader.setControl(pivot_output.withOutput(-RobotMap.GLOBAL_SPEED*2));
        }).finallyDo(() -> stopPivot());
    }

    public Command pivotDown() {
        return this.run(() -> {
            pivotLeader.setControl(pivot_output.withOutput(RobotMap.GLOBAL_SPEED*2));
        }).finallyDo(() -> stopPivot());
    }

    // --- INTAKE METHODS ---

    // Pulls the game objects into the Intake using the single Minion.
    public Command startIntake() {
        System.out.println("startIntake executed=====================");
        return this.run(() -> {
            intakeMotor.setControl(intakeOutput.withOutput(RobotMap.INTAKE_SPEED));
            goingIn = true;
        }).finallyDo(() -> stopIntake());
    }

    // Ejects the game objects from the Intake.
    public Command intakeOut() {
        System.out.println("IntakeOut executed=====================");
        return this.run(() -> {
            intakeMotor.setControl(intakeOutput.withOutput(-RobotMap.INTAKE_SPEED));
            goingIn = false;
        }).finallyDo(() -> stopIntake());
    }

    // Checks whether the intake is full or not.
    // 
    // TODO Refine the command later
    // public Command isIndexFull() {
    //     return this.run(() -> {
    //         while(!pivotUp) {
    //             while(goingIn) {
    //                 if(CANrange.getDistance().getValueAsDouble() < RobotMap.CAN_RANGE_DISTANCE_THRESHOLD) {
    //                     stopIntake();
    //                 }
    //                 else {
    //                     startIntake();
    //                 }
    //             }
    //         }
    //     });
    // }
    
    public void stopIntake() {
        System.out.println("stopIntake executed=====================");
        intakeMotor.stopMotor();
        goingIn = false;
    }

    // Completely stop everything (Useful for an emergency stop or disable command)
    public void stopPivot() {
        System.out.println("stopPivot executed=====================");
        pivotLeader.stopMotor();
        
    }

    // private boolean isPivotVoltageSpiking() {
    //     return Math.abs(pivotLeader.getMotorVoltage().getValueAsDouble()) >= RobotMap.PIVOT_VOLTAGE_SPIKE_THRESHOLD
    //         || Math.abs(pivotFollower.getMotorVoltage().getValueAsDouble()) >= RobotMap.PIVOT_VOLTAGE_SPIKE_THRESHOLD;
    // }
    private boolean isPivotStalling() {
        return Math.abs(pivotLeader.getStatorCurrent().getValueAsDouble()) >= RobotMap.PIVOT_CURRENT_STALL_THRESHOLD
            || Math.abs(pivotFollower.getStatorCurrent().getValueAsDouble()) >= RobotMap.PIVOT_CURRENT_STALL_THRESHOLD;
    }

    private void handlePivotHardStop(double expectedPosition, boolean isPivotUp) {
        stopPivot();
        pivotLeader.setPosition(expectedPosition);
        pivotFollower.setPosition(expectedPosition);
        pivotUp = isPivotUp;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Going In?", goingIn);

        // Intake Telemetry
        SmartDashboard.putNumber("Intake Battery Draw", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Draw", intakeMotor.getStatorCurrent().getValueAsDouble());
        
        // Pivot Telemetry
        SmartDashboard.putNumber("Pivot Position (Rot)", pivotLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Leader Voltage", pivotLeader.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Follower Voltage", pivotFollower.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Leader Draw", pivotLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Follower Draw", pivotFollower.getStatorCurrent().getValueAsDouble());
    } 
}