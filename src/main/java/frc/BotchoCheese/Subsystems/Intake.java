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
    private final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs();

    // Telemetry state variables
    private boolean goingIn = false;
    private boolean pivotUp = true;
    private boolean routineIntakeActive = false;
    private double lastCommandedIntakeOutput = 0.0;
    
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

        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = RobotMap.PIVOT_UP_CRUISE_VELOCITY;
        pivotMotionMagicConfigs.MotionMagicAcceleration = RobotMap.PIVOT_UP_ACCELERATION;
        pivotMotionMagicConfigs.MotionMagicJerk = RobotMap.PIVOT_UP_JERK;
        pivotConfig.MotionMagic = pivotMotionMagicConfigs;

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
        pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        // --- INTAKE CONFIGURATION (Minion) ---
        TalonFXSConfiguration intakeConfig = new TalonFXSConfiguration();
        intakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        
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
            if (isPivotVoltageSpiking()) {
                handlePivotHardStop(RobotMap.PIVOT_UP_POSITION, true);
                return;
            }

            applyPivotMotionMagicProfile(
                RobotMap.PIVOT_UP_CRUISE_VELOCITY,
                RobotMap.PIVOT_UP_ACCELERATION,
                RobotMap.PIVOT_UP_JERK
            );
            pivotLeader.setControl(pivotPositionRequest.withPosition(RobotMap.PIVOT_UP_POSITION));
            pivotUp = true;
        });
    }

    public Command setPivotDown() {
        System.out.println("setPivotDown executed=====================");
        return this.run(() -> {
            if (isPivotVoltageSpiking()) {
                handlePivotHardStop(RobotMap.PIVOT_DOWN_POSITION, false);
                return;
            }

            applyPivotMotionMagicProfile(
                RobotMap.PIVOT_DOWN_CRUISE_VELOCITY,
                RobotMap.PIVOT_DOWN_ACCELERATION,
                RobotMap.PIVOT_DOWN_JERK
            );
            pivotLeader.setControl(pivotPositionRequest.withPosition(RobotMap.PIVOT_DOWN_POSITION));
            pivotUp = false;
        });
    }

    public Command pivotUp() {
        return this.run(() -> {
            pivotLeader.setControl(pivot_output.withOutput(-RobotMap.PIVOT_MANUAL_UP_SPEED));
        }).finallyDo(() -> stopPivot());
    }

    public Command pivotDown() {
        return this.run(() -> {
            pivotLeader.setControl(pivot_output.withOutput(RobotMap.PIVOT_MANUAL_DOWN_SPEED));
        }).finallyDo(() -> stopPivot());
    }

    public Command intakeAndPivotDown() {
        return this.run(() -> {
            if (isPivotVoltageSpiking()) {
                handlePivotHardStop(RobotMap.PIVOT_DOWN_POSITION, false);
            } else {
                applyPivotMotionMagicProfile(
                    RobotMap.PIVOT_DOWN_CRUISE_VELOCITY,
                    RobotMap.PIVOT_DOWN_ACCELERATION,
                    RobotMap.PIVOT_DOWN_JERK
                );
                pivotLeader.setControl(pivotPositionRequest.withPosition(RobotMap.PIVOT_DOWN_POSITION));
                pivotUp = false;
            }
            intakeMotor.setControl(intakeOutput.withOutput(RobotMap.INTAKE_SPEED));
            goingIn = true;
            lastCommandedIntakeOutput = RobotMap.INTAKE_SPEED;
        }).finallyDo(() -> {
            stopPivot();
            stopIntake();
        });
    }

    // --- INTAKE METHODS ---

    // Pulls the game objects into the Intake using the single Minion.
    public Command startIntake() {
        System.out.println("startIntake executed=====================");
        return this.run(() -> {
            intakeMotor.setControl(intakeOutput.withOutput(RobotMap.INTAKE_SPEED));
            goingIn = true;
            lastCommandedIntakeOutput = RobotMap.INTAKE_SPEED;
        }).finallyDo(() -> stopIntake());
    }

    public Command routineIntakeOn() {
        return this.startEnd(
            () -> {
                System.out.println("routineIntakeOn started=====================");
                intakeMotor.setControl(intakeOutput.withOutput(RobotMap.INTAKE_SPEED));
                goingIn = true;
                routineIntakeActive = true;
                lastCommandedIntakeOutput = RobotMap.INTAKE_SPEED;
                System.out.println("routineIntakeOn commanded output===================== " + RobotMap.INTAKE_SPEED);
            },
            () -> {
                System.out.println("routineIntakeOn ended=====================");
                routineIntakeActive = false;
                stopIntake();
            }
        );
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
        lastCommandedIntakeOutput = 0.0;
    }

    // Completely stop everything (Useful for an emergency stop or disable command)
    public void stopPivot() {
        System.out.println("stopPivot executed=====================");
        pivotLeader.stopMotor();
    }

    public void setPivotCoastMode() {
        pivotLeader.setNeutralMode(NeutralModeValue.Coast);
        pivotFollower.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setPivotBrakeMode() {
        pivotLeader.setNeutralMode(NeutralModeValue.Brake);
        pivotFollower.setNeutralMode(NeutralModeValue.Brake);
    }

    private boolean isPivotVoltageSpiking() {
        return Math.abs(pivotLeader.getMotorVoltage().getValueAsDouble()) >= RobotMap.PIVOT_VOLTAGE_SPIKE_THRESHOLD
            || Math.abs(pivotFollower.getMotorVoltage().getValueAsDouble()) >= RobotMap.PIVOT_VOLTAGE_SPIKE_THRESHOLD;
    }

    private void handlePivotHardStop(double expectedPosition, boolean isPivotUp) {
        stopPivot();
        pivotLeader.setPosition(expectedPosition);
        pivotFollower.setPosition(expectedPosition);
        pivotUp = isPivotUp;
    }

    private void applyPivotMotionMagicProfile(double cruiseVelocity, double acceleration, double jerk) {
        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        pivotMotionMagicConfigs.MotionMagicAcceleration = acceleration;
        pivotMotionMagicConfigs.MotionMagicJerk = jerk;
        pivotLeader.getConfigurator().apply(pivotMotionMagicConfigs);
        pivotFollower.getConfigurator().apply(pivotMotionMagicConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Going In?", goingIn);
        SmartDashboard.putBoolean("Intake Routine Active", routineIntakeActive);
        SmartDashboard.putNumber("Intake Commanded Output", lastCommandedIntakeOutput);

        // Intake Telemetry
        SmartDashboard.putNumber("Intake Battery Draw", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Draw", intakeMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
        
        // Pivot Telemetry
        SmartDashboard.putNumber("Pivot Position (Rot)", pivotLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Leader Voltage", pivotLeader.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Follower Voltage", pivotFollower.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Leader Draw", pivotLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Follower Draw", pivotFollower.getStatorCurrent().getValueAsDouble());
    } 
}
