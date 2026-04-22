// currently not in use

package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;

public class CANdleLight extends SubsystemBase {
    // LED strip bounds passed into Phoenix animation/color requests.
    private static final int LED_START_INDEX = 0;
    private static final int LED_COUNT = 0;
    private static final int LED_END_INDEX = LED_START_INDEX + LED_COUNT - 1;
    private static final double CANDLE_BRIGHTNESS = 0.5;

    // Use the roboRIO CAN bus (same bus pattern as shooter TalonFX constructors).
    private final CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);

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
    }

    public void setOff() {
        setSolidColor(0, 0, 0);
    }

    public void setSolidColor(int r, int g, int b) {
        // Clear any running animation before applying a fixed RGB color.
        candle.setControl(new EmptyAnimation(0));
        candle.setControl(
            new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(r, g, b))
        );
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
        setSolidColor(0, 0, 255); //cyan
    }

    public void IndexerFullColor() {
        candle.setControl(new EmptyAnimation(1));
        setSolidColor(255, 0, 0); //red
    }
}
