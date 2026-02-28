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
    public RGBWColor currentColor = new RGBWColor(0, 0, 0);

    private CANdle candle;

    private SolidColor solidColorControl;
    private StrobeAnimation blinkingColorControl;
    

    private Cantdle() {
        this.candle = new CANdle(0, Constants.Ports.CANIVORE);
        solidColorControl = new SolidColor(0, 7);
        blinkingColorControl = new StrobeAnimation(0, 7);

        this.setToSolidColor(getAllianceColor());
    }

    public static Cantdle getInstance() {
        return (instance == null) ? instance = new Cantdle() : instance;
    }

    public Command setToSolidColor(RGBWColor color) {
        return this.runOnce(() -> {
            this.candle.setControl(solidColorControl.withColor(color));
            this.currentColor = color;
        }).withName("Set to solid " + color.toString());
    }

    public Command setToBlinkingColor(RGBWColor color) {
        return this.runOnce(() -> {
            this.candle.setControl(blinkingColorControl.withColor(color).withFrameRate(Constants.Cantdle.BLINK_FREQUENCY));
            this.currentColor = color;
        }).withName("Set to blinking " + color.toString());
    }

    public RGBWColor getAllianceColor() {
        if (Optional.of(DriverStation.Alliance.Blue).equals(Util.getAlliance())) {
            return Constants.Cantdle.BLUE;
        } else {
            return Constants.Cantdle.RED;
        }
    }

}