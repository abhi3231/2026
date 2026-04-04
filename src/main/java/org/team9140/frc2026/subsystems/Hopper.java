package org.team9140.frc2026.subsystems;

import org.team9140.frc2026.Constants;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final TalonFX spinnerMotor = new TalonFX(Constants.Ports.HOPPER_SPINNER_MOTOR, Constants.Ports.CANIVORE);
    private final TalonFX feederMotor = new TalonFX(Constants.Ports.HOPPER_FEEDER_MOTOR, Constants.Ports.CANIVORE);
    private static Hopper instance;

    private final DCMotorSim spinnerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1));

    private final DCMotorSim feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1));

    private Hopper() {
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
                .withInverted(InvertedValue.Clockwise_Positive);

        TalonFXConfiguration spinnerMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(spinnerCurrentLimits)
                .withMotorOutput(spinnerMotorOutputConfig);

        TalonFXConfiguration feederMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(feederCurrentLimits)
                .withMotorOutput(feederMotorOutputConfig);

        this.spinnerMotor.getConfigurator().apply(spinnerMotorConfigs);
        this.feederMotor.getConfigurator().apply(feederMotorConfigs);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.setDefaultCommand(off());
    }

    public static Hopper getInstance() {
        return (instance == null) ? instance = new Hopper() : instance;
    }

    private Command setSpeeds(double spinnerVoltage, double feederVoltage) {
        return this.runOnce(() -> this.spinnerMotor.setVoltage(spinnerVoltage))
                .andThen(this.runOnce(() -> this.feederMotor.setVoltage(feederVoltage)))
                .andThen(Commands.idle(this));
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

    public Command reverseAndOff() {
        return this.setSpeeds(0, -Constants.Hopper.FEEDER_VOLTAGE).withTimeout(Constants.Hopper.REVERSE_FEEDER_TIME).andThen(off())
                .withName("Reverse Hopper and Off");
    }

    private static final double kSimLoopPeriod = Constants.Intake.SIM_PERIOD; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void updateSimState(double deltaTime, double batteryVoltage) {
        double spinnerMotor = this.spinnerMotor.getSimState().getMotorVoltage();
        SmartDashboard.putNumber("spinner volts", spinnerMotor);
        this.spinnerSim.setInputVoltage(spinnerMotor);
        this.spinnerSim.update(deltaTime);
        this.spinnerMotor.getSimState().setRawRotorPosition(this.spinnerSim.getAngularPositionRotations());
        this.spinnerMotor.getSimState().setRotorVelocity(this.spinnerSim.getAngularVelocityRPM() / 60.0);
        this.spinnerMotor.getSimState().setRotorAcceleration(
                this.spinnerSim.getAngularAccelerationRadPerSecSq() / 2.0 / Math.PI);
        
        double feederMotorVolts = this.feederMotor.getSimState().getMotorVoltage();
        SmartDashboard.putNumber("feeder volts", feederMotorVolts);
        this.spinnerSim.setInputVoltage(feederMotorVolts);
        this.spinnerSim.update(deltaTime);
        this.spinnerMotor.getSimState().setRawRotorPosition(this.feederSim.getAngularPositionRotations());
        this.spinnerMotor.getSimState().setRotorVelocity(this.feederSim.getAngularVelocityRPM() / 60.0);
        this.spinnerMotor.getSimState().setRotorAcceleration(
                this.feederSim.getAngularAccelerationRadPerSecSq() / 2.0 / Math.PI);
    }

}
