//currently not in use

package frc.BotchoCheese.Commands;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CalibrateSwerveOffsets extends Command {

    private final CANcoder[] cancoderArray;
    private final String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};

    /**
     * Command to calibrate swerve CANcoder MagnetOffsets.
     * Pass your 4 CANcoders from TunerConstants or your drivetrain.
     */
    public CalibrateSwerveOffsets(CANcoder frontLeft, CANcoder frontRight, CANcoder backLeft, CANcoder backRight) {
        this.cancoderArray = new CANcoder[]{frontLeft, frontRight, backLeft, backRight};
    }

    @Override
    public void initialize() {
        if (DriverStation.isEnabled()) {
            System.out.println("WARNING: Robot is ENABLED. Calibration is safer in DISABLED mode.");
        }
        System.out.println("=== Starting Swerve Offset Calibration ===");
        System.out.println("Ensure all wheels are mechanically aligned straight forward.");
        System.out.println("Use a straight edge/jig and make bevel gears point the same direction (e.g., left).");
    }

    @Override
    public void execute() {
        for (int i = 0; i < cancoderArray.length; i++) {
            CANcoder coder = cancoderArray[i];
            String name = moduleNames[i];

            // Read raw absolute position (in rotations, typically -0.5 to +0.5 range)
            double rawAbsRot = coder.getAbsolutePosition().getValueAsDouble();

            // Compute offset: negative so current aligned position becomes ~0
            double computedOffset = -rawAbsRot;

            // Create and apply config (persistent on hardware!)
            CANcoderConfiguration config = new CANcoderConfiguration();

            config.MagnetSensor.MagnetOffset = computedOffset;

            // Use the real field: discontinuity point (controls effective range)
            // 0.5 = signed [-0.5, +0.5) — standard for most FRC swerve
            config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

            // Sensor direction (adjust if your steering is inverted after test)
            config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            coder.getConfigurator().apply(config);

            // Log to RioLog / console
            System.out.printf("%s - Raw Abs: %.6f rot | Applied MagnetOffset: %.6f rot | Discontinuity: %.1f%n", 
                              name, rawAbsRot, computedOffset, config.MagnetSensor.AbsoluteSensorDiscontinuityPoint);

            // Publish to dashboard for easy verification
            SmartDashboard.putNumber(name + " Raw Abs (rot)", rawAbsRot);
            SmartDashboard.putNumber(name + " Applied Offset (rot)", computedOffset);
            SmartDashboard.putNumber(name + " Discontinuity Point", config.MagnetSensor.AbsoluteSensorDiscontinuityPoint);
        }

        System.out.println("=== Calibration Complete! ===");
        System.out.println("Offsets are now saved directly to CANcoder hardware.");
        System.out.println("Test: Command 0° in code — wheels should stay perfectly straight.");
        System.out.println("No need to update TunerConstants offsets (keep them at 0).");
    }

    @Override
    public boolean isFinished() {
        return true; // One-shot command
    }

    @Override
    public boolean runsWhenDisabled() {
        return true; //
    } 
}