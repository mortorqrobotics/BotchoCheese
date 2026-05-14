// currently not in use

package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import static edu.wpi.first.units.Units.Hertz;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchoCheese.Constants.RobotMap;

public class CANdleLight extends SubsystemBase {
    // LED strip bounds passed into Phoenix animation/color requests.
    private static final int LED_START_INDEX = 0;
    private static final int LED_COUNT = 0;
    private static final int LED_END_INDEX = LED_START_INDEX + LED_COUNT - 1;
    private static final double CANDLE_BRIGHTNESS = 0.5;

    // Use the roboRIO CAN bus (same bus pattern as shooter TalonFX constructors).
    private final CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);
    private final ColorFlowAnimation m_slot0Animation = new ColorFlowAnimation(0, 0)
        .withSlot(0)
        .withColor(rgbOnly(255, 255, 255))
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Hertz.of(25));
     private final SolidColor[] m_colors = new SolidColor[] {};

    public CANdleLight() {
        // Configure the CANdle once at startup, then leave color changes to the helper methods below.
        CANdleConfiguration config = new CANdleConfiguration();
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = CANDLE_BRIGHTNESS;
        config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;

        

        candle.getConfigurator().apply(config);
        setOff();
        setDefaultCommand(updateLEDs());
    }

    public void setOff() {
        setSolidColor(0, 0, 0);
    }

    public void setSolidColor(int r, int g, int b) {
        // Clear any running animation before applying a fixed RGB color.
        candle.setControl(new EmptyAnimation(0));
        candle.setControl(
            new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(rgbOnly(r, g, b))
        );
    }

    private static RGBWColor rgbOnly(int r, int g, int b) {
        // Phoenix SolidColor uses RGBWColor, but this robot is configured for an RGB strip.
        // Keep the white channel at 0 so solid colors are driven as RGB only.
        return new RGBWColor(r, g, b, 0);
    }

    public void setRainbow() {
        RainbowAnimation rainbow = new RainbowAnimation(LED_START_INDEX, LED_END_INDEX);
        rainbow.Slot = 0;
        rainbow.Brightness = 1.0;
        rainbow.FrameRate = 120.0;
        rainbow.UpdateFreqHz = 0.0;
        candle.setControl(rainbow);
    }

    public void setDisabledColor() {
        setSolidColor(255, 80, 0); //orange
    }

    public void setAutonomousColor() {
        setSolidColor(255, 255, 0); //yellow
    }

    public void setTeleopColor() {
        setSolidColor(0, 255, 0); //lime
    }

    public void setTestColor() {
        setSolidColor(0, 0, 255); // blue
    }

    public void IndexerFullColor() {
        candle.setControl(new EmptyAnimation(1));
        setSolidColor(255, 0, 0); //red
    }

    public Command updateLEDs() {
        return run(() -> {
            for (var solidColor : m_colors) {
                candle.setControl(solidColor);
            }
            candle.setControl(m_slot0Animation);
        });
    }
}
// package frc.BotchoCheese.Subsystems;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.RGBWColor;

// /**
//  * Subsystem that controls an addressable LED strip using a CANdle.
//  */
// public class CANdleLight extends SubsystemBase {
//     private final CANBus kCANBus = new CANBus("rio");
//     private final CANdle m_candle = new CANdle(41, kCANBus);

//     private final SolidColor[] m_colors = new SolidColor[] {
        
//     };

//     public CANdleLight() {
//         setDefaultCommand(updateLEDs());
//     }

//     /**
//      * Updates the animations and LEDs of the CANdle.
//      *
//      * @return Command to run
//      */
//     public Command updateLEDs() {
//         return run(() -> {
//             for (var solidColor : m_colors) {
//                 m_candle.setControl(solidColor);
//             }
//         });
//     }
// }
