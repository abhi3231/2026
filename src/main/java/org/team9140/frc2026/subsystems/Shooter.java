package org.team9140.frc2026.subsystems;

import java.util.function.Supplier;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.helpers.AimAlign;
import org.team9140.lib.Util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {
    private final TalonFX yawMotor = new TalonFX(Constants.Ports.YAW_MOTOR, Constants.Ports.CANIVORE);
    private final TalonFX shooterMotor = new TalonFX(Constants.Ports.SHOOTER_MOTOR, Constants.Ports.CANIVORE);

    private double yawTargetPosition = 0;
    private double shooterTargetVelocity = 0;

    private final MotionMagicTorqueCurrentFOC yawMotorControl = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    private final VelocityTorqueCurrentFOC shooterSpeedControl = new VelocityTorqueCurrentFOC(0).withSlot(0);

    private static Shooter instance;

    public static Shooter getInstance() {
        return (instance == null) ? instance = new Shooter() : instance;
    }

    private Shooter() {
        MotionMagicConfigs yawMMConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(Constants.Shooter.YAW_ACCELERATION)
                .withMotionMagicCruiseVelocity(Constants.Shooter.YAW_CRUISE_VELOCITY);

        Slot0Configs yawSlot0Configs = new Slot0Configs()
                .withKS(Constants.Shooter.YAW_KS)
                .withKV(Constants.Shooter.YAW_KV)
                .withKA(Constants.Shooter.YAW_KA)
                .withKP(Constants.Shooter.YAW_KP)
                .withKI(Constants.Shooter.YAW_KI)
                .withKD(Constants.Shooter.YAW_KD);

        MotorOutputConfigs yawMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        yawMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

        Slot0Configs shooterSlot0Configs = new Slot0Configs()
                .withKS(Constants.Shooter.SHOOTER_KS)
                .withKV(Constants.Shooter.SHOOTER_KV)
                .withKA(Constants.Shooter.SHOOTER_KA)
                .withKP(Constants.Shooter.SHOOTER_KP)
                .withKI(Constants.Shooter.SHOOTER_KI)
                .withKD(Constants.Shooter.SHOOTER_KD);

        MotorOutputConfigs shooterMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        shooterMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;

        TorqueCurrentConfigs shooterTorqueCurrentConfigs = new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Constants.Shooter.PEAK_FORWARD_TORQUE)
                .withPeakReverseTorqueCurrent(0.0);

        TalonFXConfiguration yawConfig = new TalonFXConfiguration()
                .withMotionMagic(yawMMConfigs)
                .withSlot0(yawSlot0Configs)
                .withMotorOutput(yawMotorOutputConfigs);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                .withSlot0(shooterSlot0Configs)
                .withMotorOutput(shooterMotorOutputConfigs)
                .withTorqueCurrent(shooterTorqueCurrentConfigs);

        yawMotor.getConfigurator().apply(yawConfig);
        shooterMotor.getConfigurator().apply(shooterConfig);

        Mechanism2d yawMech = new Mechanism2d(1, 1);
        MechanismRoot2d yawRoot = yawMech.getRoot("yawArm Root", 1.5, 0.5);
        yawArmLigament = yawRoot.append(new MechanismLigament2d(
                "yawArm",
                0.3,
                0,
                6,
                new Color8Bit(Color.kYellow)));

        SmartDashboard.putData("YAW ARM MECHANISM", yawMech);

        this.setDefaultCommand(this.off());

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private boolean isManual = false;

    public Command manualAdjust(boolean left) {
        return this.runOnce(() -> {
            this.isManual = true;
            this.shooterMotor.setControl(new VoltageOut(12.0));
            this.yawMotor.setControl(new VoltageOut(
                    left ? Constants.Shooter.ADJUST_VOLTAGE : -Constants.Shooter.ADJUST_VOLTAGE));
        }).andThen(this.run(() -> {})).finallyDo(() -> {
            this.yawMotor.setControl(new StaticBrake());
        }).withName("Adjust Manually");
    }

    public Command manualLeft() {
        return manualAdjust(true)
                .withName("Manually Adjust to Left");
    }

    public Command manualRight() {
        return manualAdjust(false)
                .withName("Manually Adjust to Right");
    }

    /*
     * 1. get target and chassis state from suppliers
     * 2. calculate distance for required flywheel speed
     * 3. calculate turret angle to make it to spot
     * 4. set yaw and flywheel motors
     */
    public Command aim(Supplier<SwerveDriveState> chassisStateSupplier) {
        return this.run(() -> {
            if (this.isManual)
                return;
            Pose2d chassisPose = chassisStateSupplier.get().Pose;
            Translation2d targetPose = AimAlign.getZone(chassisPose).getTranslation();
            this.shooterMotor.setControl(shooterSpeedControl.withVelocity(
                    AimAlign.getRequiredSpeed(chassisPose, targetPose)));
            this.yawMotor.setControl(yawMotorControl.withPosition(
                    AimAlign.yawAngleToPos(chassisPose, targetPose) / (2.0 * Math.PI)));
        }).withName("Continuously Aim Automatically");
    }

    public Command idle() {
        return this.runOnce(() -> {
            if (this.isManual)
                return;
            // point turret forward / starting orientation / whatever
            this.yawMotor.setControl(yawMotorControl.withPosition(0));
            this.shooterMotor.setControl(new VoltageOut(Constants.Shooter.IDLE_VOLTAGE));
        }).andThen(this.run(() -> {})).withName("Idle");
    }

    // make this default command
    public Command off() {
        return this.runOnce(() -> {
            if (this.isManual) 
                return;
            this.shooterMotor.setControl(new CoastOut());
            this.yawMotor.setControl(new StaticBrake());
        }).withName("Shooter Off");
    }

    @Override
    public void periodic() {
        // all these are in rotations per second
        SmartDashboard.putNumber("Yaw Angle", yawMotor.getPosition(true).getValueAsDouble());
        SmartDashboard.putNumber("Yaw Target Position", this.yawTargetPosition / Math.PI / 2.0);
        SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getVelocity(true).getValueAsDouble());
        SmartDashboard.putNumber("Shooter Target Velocity", this.shooterTargetVelocity / Math.PI / 2.0);
    }

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SingleJointedArmSim yawMotorSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            60,
            1,
            0.2,
            -Math.PI,
            Math.PI,
            false,
            0);
    private final MechanismLigament2d yawArmLigament;

    private TalonFXSimState shooterMotorSimState;
    private final FlywheelSim shooterMotorSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(2), 0.2, 60),
            DCMotor.getKrakenX60Foc(2));

    private final StructPublisher<Pose3d> publisher1 = NetworkTableInstance.getDefault()
            .getStructTopic("shooter", Pose3d.struct).publish();
    private Pose3d shooterPos = new Pose3d();

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime);
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void updateSimState(double deltatime) {
        TalonFXSimState yawMotorSimState = yawMotor.getSimState();
        double yawSimVolts = yawMotorSimState.getMotorVoltage();
        yawMotorSim.setInputVoltage(yawSimVolts);
        yawMotorSim.update(deltatime);

        yawMotor.getPosition().refresh();
        yawArmLigament.setAngle(yawMotor.getPosition().getValueAsDouble() * 360);// convert rot to deg

        yawMotorSimState.setRawRotorPosition(yawMotorSim.getAngleRads() / 2.0 / Math.PI);
        yawMotorSimState.setRotorVelocity(yawMotorSim.getVelocityRadPerSec() / 2.0 / Math.PI);

        shooterPos = new Pose3d(0, 0, 0,
                new Rotation3d(0, 0, yawMotor.getPosition().getValueAsDouble() * 2 * Math.PI));
        publisher1.set(shooterPos);

        shooterMotorSimState = shooterMotor.getSimState();
        double shooterSimVolts = shooterMotorSimState.getMotorVoltage();
        shooterMotorSim.setInputVoltage(shooterSimVolts);

        shooterMotorSim.update(deltatime);

        shooterMotorSimState.setRotorVelocity(shooterMotorSim.getAngularVelocityRPM() / 60.0);
        shooterMotorSimState.setRotorAcceleration(
                shooterMotorSim.getAngularAccelerationRadPerSecSq() / 2.0 / Math.PI);
    }

    public final Trigger yawIsAtPosition = new Trigger(
            () -> Util.epsilonEquals(this.yawMotor.getPosition(false).getValueAsDouble(),
                    this.yawTargetPosition));

    public final Trigger shooterIsAtVelocity = new Trigger(
            () -> Util.epsilonEquals(this.shooterMotor.getVelocity(false).getValueAsDouble(),
                    this.shooterTargetVelocity));
}