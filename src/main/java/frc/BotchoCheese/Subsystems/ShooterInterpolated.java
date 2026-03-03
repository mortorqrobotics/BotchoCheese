package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.BotchoCheese.Robot;
import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Constants.RobotMap; // Assuming your IDs are here

public class ShooterInterpolated extends SubsystemBase {
    // Calibrated lookup points (distance in meters -> setpoint).
    // Replace with measured values after tuning.
    private static final double[] DISTANCE_POINTS_METERS = { 2.0, 5.0, 10.0 };
    private static final double[] SHOOTER_SPEED_POINTS = { 0.25, 0.50, 0.75 };
    private static final double[] HOOD_POSITION_POINTS = { 0.25, 0.50, 0.75 };

    // Motor controllers
    private final TalonFX leftShooter;
    private final TalonFX middleShooter;
    private final TalonFX rightShooter;
    private final TalonFXS hood;

    // Control requests (Phoenix 6 uses request objects instead of passing doubles directly)
    private final DutyCycleOut m_output = new DutyCycleOut(0);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);

    private boolean shooterTurning = false;
    private boolean hoodDown = false;
    
    public ShooterInterpolated() {
        leftShooter = new TalonFX(RobotMap.LEFT_SHOOTER_MOTOR_ID);
        middleShooter = new TalonFX(RobotMap.MIDDLE_SHOOTER_MOTOR_ID);
        rightShooter = new TalonFX(RobotMap.RIGHT_SHOOTER_MOTOR_ID);
        hood = new TalonFXS(RobotMap.HOOD_MOTOR_ID);

        // Apply basic configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();

        // PID Shooter Values
        // in init function, set slot 0 gains
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = RobotMap.SHOOTER_P_VALUE;
        slot0Configs.kI = RobotMap.SHOOTER_I_VALUE;
        slot0Configs.kD = RobotMap.SHOOTER_D_VALUE;
        config.Slot0 = slot0Configs;
        //https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html

        // PID Hood Values
        // in init function, set slot 0 gains
        Slot0Configs slot1Configs = new Slot0Configs();
        slot1Configs.kP = RobotMap.HOOD_P_VALUE;
        slot1Configs.kI = RobotMap.HOOD_I_VALUE;
        slot1Configs.kD = RobotMap.HOOD_D_VALUE;
        hoodConfig.Slot0 = slot1Configs;
        
        // Verify
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0; // Minions generally stay in the 30-50A range
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 30.0; 
        currentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits = currentLimits;
        
        /* Set motors to Brake mode so the climber doesn't slide down */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftShooter.getConfigurator().apply(config);
        middleShooter.getConfigurator().apply(config);
        rightShooter.getConfigurator().apply(config);
        hood.getConfigurator().apply(hoodConfig);
    }

    // Moves the Shooter counter Clockwise.
    
    // public Command turnShooters() {
    //     return this.run(() -> {
    //         leftShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
    //         middleShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
    //         rightShooter.setControl(m_output.withOutput(RobotMap.MOTOR_SHOOTER_TOP_OUT_SPEED));
    //         shooterTurning = true;
    //     }).finallyDo(() -> stopMotors());
    // }

    //1. Use tx and ty to get distance from the robot and the tag
    //2. Determine whether the distance is "close," "medium," or "far" away from the tag
    //3. Change the speed accordingly for each of these situations

    public Command hoodUp() {
        return this.run(() -> {
            hood.setControl(m_output.withOutput(RobotMap.HOOD_SPEED));
            hoodDown = false;
        }).finallyDo(() -> stopMotors());
    }

    public Command hoodDown() {
        return this.run(() -> {
            hood.setControl(m_output.withOutput(-RobotMap.HOOD_SPEED));
            hoodDown = true;
        }).finallyDo(() -> stopMotors());
    }

    public Command shoot() {
        // We use startEnd so it automatically stops motors when the command finishes (button release)
        return this.startEnd(
            // When command starts/runs:
            () -> {
                double distance = getCurrentDistanceMeters();
                double shooterSpeed = interpolate(distance, DISTANCE_POINTS_METERS, SHOOTER_SPEED_POINTS);
                double hoodPosition = interpolate(distance, DISTANCE_POINTS_METERS, HOOD_POSITION_POINTS);

                // Safety clamp for motor output and hood target.
                shooterSpeed = MathUtil.clamp(shooterSpeed, 0.0, 1.0);
                hoodPosition = MathUtil.clamp(hoodPosition, 0.0, 1.0);

                hood.setControl(hoodPositionRequest.withPosition(hoodPosition));
                leftShooter.set(shooterSpeed);
                middleShooter.set(shooterSpeed);
                rightShooter.set(shooterSpeed);
            },
            // When command ends:
            () -> {
                stopMotors();
            }
        );
    }

    public void stopMotors() {
        leftShooter.stopMotor();
        middleShooter.stopMotor();
        rightShooter.stopMotor();
        hood.stopMotor();
        shooterTurning = false;
    }

    /**
     * Update shooter speed based on distance from target
     */
    public void updateSpeed() {
        double distance = getCurrentDistanceMeters();
        RobotMap.SHOOTER_SPEED = MathUtil.clamp(
            interpolate(distance, DISTANCE_POINTS_METERS, SHOOTER_SPEED_POINTS),
            0.0, 1.0
        );
    }

    public void updateHoodPosition() {
        double distance = getCurrentDistanceMeters();
        RobotMap.HOOD_POSITION = MathUtil.clamp(
            interpolate(distance, DISTANCE_POINTS_METERS, HOOD_POSITION_POINTS),
            0.0, 1.0
        );
    }

    /**
     * Calculates optimal shooter speed based on a preset slope-intercept formula
     */
    public void regressionSpeedShooter() {
        double distance = getCurrentDistanceMeters();

        // y = mx + b
        RobotMap.SHOOTER_SPEED = MathUtil.clamp(
            RobotMap.SHOOTER_SPEED_REGRESSION_SLOPE * distance + RobotMap.SHOOTER_SPEED_REGRESSION_Y_INTERCEPT,
            0.0, 1.0
        );
    }

    public void regressionAngleHood() {
        double distance = getCurrentDistanceMeters();

        // y = mx + b
        RobotMap.HOOD_POSITION = MathUtil.clamp(
            RobotMap.HOOD_SPEED_REGRESSION_SLOPE * distance + RobotMap.HOOD_SPEED_REGRESSION_Y_INTERCEPT,
            0.0, 1.0
        );
    }

    private double getCurrentDistanceMeters() {
        LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(RobotMap.LIMELIGHT_NAME);
        if (rawFiducials.length == 0) {
            // Fallback distance when vision is unavailable.
            return DISTANCE_POINTS_METERS[0];
        }

        double bestDistance = rawFiducials[0].distToRobot;
        for (int i = 1; i < rawFiducials.length; i++) {
            if (rawFiducials[i].distToRobot < bestDistance) {
                bestDistance = rawFiducials[i].distToRobot;
            }
        }
        return bestDistance;
    }

    private static double interpolate(double x, double[] xPoints, double[] yPoints) {
        if (xPoints.length != yPoints.length || xPoints.length < 2) {
            throw new IllegalArgumentException("Interpolation tables must have matching lengths >= 2");
        }

        if (x <= xPoints[0]) {
            return yPoints[0];
        }
        if (x >= xPoints[xPoints.length - 1]) {
            return yPoints[yPoints.length - 1];
        }

        for (int i = 0; i < xPoints.length - 1; i++) {
            double x0 = xPoints[i];
            double x1 = xPoints[i + 1];
            if (x >= x0 && x <= x1) {
                double y0 = yPoints[i];
                double y1 = yPoints[i + 1];
                double t = (x - x0) / (x1 - x0);
                return y0 + t * (y1 - y0);
            }
        }

        return yPoints[yPoints.length - 1];
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("kP", RobotMap.SHOOTER_P_VALUE);
        SmartDashboard.putNumber("kI", RobotMap.SHOOTER_I_VALUE);
        SmartDashboard.putNumber("kD", RobotMap.SHOOTER_D_VALUE);
        SmartDashboard.putBoolean("Shooter Turning?", shooterTurning);
        SmartDashboard.putBoolean("Hood Going Down?", hoodDown);
        
        SmartDashboard.putNumber("Shooter Battery Draw", leftShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor Draw", leftShooter.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood Battery Draw", hood.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood Motor Draw", hood.getStatorCurrent().getValueAsDouble());
    } 
}

