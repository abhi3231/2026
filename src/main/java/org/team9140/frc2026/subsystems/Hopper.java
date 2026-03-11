package org.team9140.frc2026.subsystems;

import org.team9140.frc2026.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase{
    private final TalonFX spinnerMotor; // This is the spinny thingy
    private final TalonFX feederMotor; // This feeds to shooter
    private static Hopper instance;

    private Hopper() {
        this.spinnerMotor = new TalonFX(Constants.Ports.HOPPER_SPINNER_MOTOR, Constants.Ports.CANIVORE);
        this.feederMotor = new TalonFX(Constants.Ports.HOPPER_FEEDER_MOTOR, Constants.Ports.CANIVORE);
        
        CurrentLimitsConfigs spinnerCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Hopper.SPINNER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Hopper.SPINNER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        CurrentLimitsConfigs feederCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Hopper.FEEDER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Hopper.FEEDER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        
        MotorOutputConfigs spinnerMotorOutputConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        MotorOutputConfigs feederMotorOutputConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        TalonFXConfiguration spinnerMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(spinnerCurrentLimits)
                .withMotorOutput(spinnerMotorOutputConfig);

        TalonFXConfiguration feederMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(feederCurrentLimits)
                .withMotorOutput(feederMotorOutputConfig);

        this.spinnerMotor.getConfigurator().apply(spinnerMotorConfigs);
        this.feederMotor.getConfigurator().apply(feederMotorConfigs);
    }

    public static Hopper getInstance() {
        return (instance == null) ? instance = new Hopper() : instance;
    }

    private Command setSpeeds(double spinnerVoltage, double feederVoltage) {
        return this.runOnce(() -> this.spinnerMotor.setVoltage(spinnerVoltage))
            .andThen(this.runOnce(() -> this.feederMotor.setVoltage(feederVoltage)))
                .withName("Set Speed to Hopper");
    }

    public Command feed() {
        return this.setSpeeds(Constants.Hopper.SPINNER_VOLTAGE, Constants.Hopper.FEEDER_VOLTAGE)
                .withName("Hopper Feeder");
    }

    public Command unjam() {
        return this.setSpeeds(-Constants.Hopper.SPINNER_VOLTAGE, -Constants.Hopper.FEEDER_VOLTAGE)
                .withName("Unjam Hopper");
    }

    public Command off() {
        return this.setSpeeds(0, 0)
                .withName("Hopper Off");
    }
}
