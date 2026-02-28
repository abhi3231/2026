package org.team9140.frc2026.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class Cantdle extends SubsystemBase {
    private static Cantdle instance;

    public static final RGBWColor RED = new RGBWColor(255, 0, 0);
    public static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
    public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
    public static final RGBWColor PINK = new RGBWColor(245, 110, 229);
    public static final RGBWColor ORANGE = new RGBWColor(255, 157, 0);
    public static final RGBWColor PURPLE = new RGBWColor(227, 18, 254);
    public static final RGBWColor OFF = new RGBWColor(0, 0, 0);
    public RGBWColor current = new RGBWColor(0, 0, 0);

    private CANdle candle;

    private SolidColor solidColorControl;
    private StrobeAnimation blinkingColorControl;
    

    private Cantdle() {
        this.candle = new CANdle(0, Constants.Ports.CANIVORE);
        solidColorControl = new SolidColor(0, 7);
        blinkingColorControl = new StrobeAnimation(0, 7);

        this.setSolidColor(getAllianceColor());
    }

    public static Cantdle getInstance() {
        return (instance == null) ? instance = new Cantdle() : instance;
    }

    public Command setSolidColor(RGBWColor color) {
        return this.runOnce(() -> {
            this.candle.setControl(solidColorControl.withColor(color));
            this.current = color;
        }).withName("Set to solid " + color.toString());
    }

    public Command setBlinkingColor(RGBWColor color) {
        return this.runOnce(() -> {
            this.candle.setControl(blinkingColorControl.withColor(color).withFrameRate(1));
            this.current = color;
        }).withName("Set to blinking " + color.toString());
    }

    public RGBWColor getAllianceColor() {
        if (Optional.of(DriverStation.Alliance.Blue).equals(Util.getAlliance())) {
            return BLUE;
        } else {
            return RED;
        }
    }

}