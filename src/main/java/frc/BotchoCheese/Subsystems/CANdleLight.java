package frc.BotchoCheese.Subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.CANBus;
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
    private static final int LED_START_INDEX = RobotMap.CANDLE_LED_START_INDEX;
    private static final int LED_END_INDEX =
        RobotMap.CANDLE_LED_START_INDEX + RobotMap.CANDLE_LED_COUNT - 1;
    private final CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID, new CANBus(RobotMap.CANDLE_CAN_BUS));

    public CANdleLight() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = RobotMap.CANDLE_BRIGHTNESS;
        config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;
        candle.getConfigurator().apply(config);
        setOff();
    }

    public void setOff() {
        setSolidColor(0, 0, 0);
    }

    public void setSolidColor(int r, int g, int b) {
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

    // TODO May not need
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
