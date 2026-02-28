package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

public class Intake extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final TalonFX extendMotor;
    private static Intake instance;
    private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0);

    Mechanism2d intakeMechanism = new Mechanism2d(Constants.Intake.MECHANISM_LENGTH, Constants.Intake.MECHANISM_HEIGHT);
    MechanismRoot2d root = intakeMechanism.getRoot("root", 1, 1);
    MechanismLigament2d intakeSlide = root
            .append(new MechanismLigament2d("intakeSlide", Constants.Intake.LIGAMENT_LENGTH, 0));

    private Intake() {
        this.rollerMotor = new TalonFX(Constants.Ports.INTAKE_SPIN_MOTOR, Constants.Ports.CANIVORE);
        this.extendMotor = new TalonFX(Constants.Ports.INTAKE_EXTEND_MOTOR, Constants.Ports.CANIVORE);

        CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.ROLLER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Intake.ROLLER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        CurrentLimitsConfigs extendCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.EXTEND_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Intake.EXTEND_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs spinMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        MotorOutputConfigs extendMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        this.extendMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Intake.MOTION_MAGIC_CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.Intake.MOTION_MAGIC_ACCELERATION);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Intake.FORWARD_SOFT_LIMIT_THRESHOLD)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.Intake.REVERSE_SOFT_LIMIT_THRESHOLD)
                .withReverseSoftLimitEnable(true);

        TalonFXConfiguration spinMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(rollerCurrentLimits)
                .withMotorOutput(spinMotorOutputConfigs);

        TalonFXConfiguration extendMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(extendCurrentLimits)
                .withMotorOutput(extendMotorOutputConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                .withSlot0(new Slot0Configs().withKP(0.5))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Intake.EXTENSION_GEAR_RATIO));

        this.rollerMotor.getConfigurator().apply(spinMotorConfigs);
        this.extendMotor.getConfigurator().apply(extendMotorConfigs);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public static Intake getInstance() {
        return (instance == null) ? instance = new Intake() : instance;
    }

    @Override
    public void periodic() {
        this.extendMotor.getPosition().refresh();
    }

    double targetPosition = 0;

    /**
     * @return how many meters extended out is the thing
     */
    public double getPosition() {
        return Constants.Intake.PINION_CIRCUMFERENCE * this.extendMotor.getPosition(false).getValueAsDouble();
    }

    /**
     * @param position Target extension in meters
     * @return command that moves the arm to the given position
     */
    public Command setPosition(double position) {
        return this.runOnce(() -> {
            this.targetPosition = position;
            this.extendMotor.setControl(
                    this.motionMagic.withPosition(this.targetPosition / Constants.Intake.PINION_CIRCUMFERENCE));
        }).withName("Set Intake Position");
    }

    public final Trigger atPosition = new Trigger(
            () -> Util.epsilonEquals(getPosition(), this.targetPosition,
                    Constants.Intake.TOLERANCE));

    /**
     * @return command that moves the arm to the "in" position (meters)
     */
    public Command armIn() {
        return this.setPosition(Constants.Intake.ARM_IN_POSITION)
                .withName("Set Intake position to Arm In");
    }

    /**
     * @return command that moves the arm to the "out" position (meters)
     */
    public Command armOut() {
        return this.setPosition(Constants.Intake.ARM_OUT_POSITION)
                .withName("Set Intake position to Arm Out");
    }

    public Command setRollerSpeed(double speed) {
        return this.runOnce(() -> rollerMotor.set(speed))
                .withName("Set Intake Arm Roller Speed");
    }

    public Command off() {
        return this.runOnce(() -> {
            setRollerSpeed(Constants.Intake.INTAKE_OFF);
        })
                .withName("Set Intake Arm Roller Speed to Off");
    }

    public Command intake() {
        return this.armOut().andThen(this.runOnce(() -> {
            setRollerSpeed(Constants.Intake.INTAKE_VOLTAGE);
        }))
                .withName("Intake");
    }

    public Command reverse() {
        return this.armOut().andThen(this.runOnce(() -> {
            setRollerSpeed(-Constants.Intake.INTAKE_VOLTAGE);
        }))
                .withName("Reverse intake");
    }

    public Command armInOutLoop() {
        return this.run(() -> {
            armIn().withTimeout(1)
                    .andThen(armOut().withTimeout(1))
                    .repeatedly();
        })
                .withName("Intake Arm In and Out position repeatedly");
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

    ElevatorSim extensionSim = new ElevatorSim(DCMotor.getKrakenX44(1),
            Constants.Intake.EXTENSION_GEAR_RATIO,
            1,
            Constants.Intake.PINION_CIRCUMFERENCE / Math.PI / 2.0,
            Constants.Intake.MIN_HEIGHT,
            Constants.Intake.MAX_HEIGHT,
            false,
            0);

    private void updateSimState(double t, double volts) {
        double extendVolts = this.extendMotor.getSimState().getMotorVoltage();
        SmartDashboard.putNumber("extend volts", extendVolts);
        this.extensionSim.setInputVoltage(extendVolts);
        this.extensionSim.update(t);

        double pos = this.extensionSim.getPositionMeters();
        double vel = this.extensionSim.getVelocityMetersPerSecond();

        this.extendMotor.getSimState().setRawRotorPosition(
                pos / Constants.Intake.PINION_CIRCUMFERENCE * Constants.Intake.EXTENSION_GEAR_RATIO);
        this.extendMotor.getSimState()
                .setRotorVelocity(vel / Constants.Intake.PINION_CIRCUMFERENCE * Constants.Intake.EXTENSION_GEAR_RATIO);

        intakeSlide.setLength(0.5 + pos);
    }
}