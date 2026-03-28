package org.team9140.frc2026.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Turret;
import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.helpers.AimAlign;
import org.team9140.lib.Util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
    private final TalonFX shooterFollower = new TalonFX(Constants.Ports.SHOOTER_FOLLOWER_MOTOR,
            Constants.Ports.CANIVORE);

    private final MotionMagicTorqueCurrentFOC yawMotorControl = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    private final VelocityTorqueCurrentFOC shooterSpeedControl = new VelocityTorqueCurrentFOC(0).withSlot(0);

    private static Shooter instance;

    public static Shooter getInstance() {
        return (instance == null) ? instance = new Shooter() : instance;
    }

    private Shooter() {
        // START TURRET CONFIG

        MotionMagicConfigs turretMotionMagicConfig = new MotionMagicConfigs()
                .withMotionMagicAcceleration(Constants.Turret.MM_ACCELERATION)
                .withMotionMagicCruiseVelocity(Constants.Turret.MM_CRUISE_VELOCITY);

        Slot0Configs turretSlot0Configs = new Slot0Configs()
                .withKS(Constants.Turret.KS)
                .withKV(Constants.Turret.KV)
                .withKA(Constants.Turret.KA)
                .withKP(Constants.Turret.KP)
                .withKI(Constants.Turret.KI)
                .withKD(Constants.Turret.KD);

        MotorOutputConfigs turretMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        TorqueCurrentConfigs turretTorqueCurrentConfigs = new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Constants.Turret.STATOR_CURRENT_LIMIT)
                .withPeakReverseTorqueCurrent(-Constants.Turret.STATOR_CURRENT_LIMIT);

        CurrentLimitsConfigs turretCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Turret.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Turret.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        SoftwareLimitSwitchConfigs turretSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Turret.FORWARD_SOFT_LIMIT_THRESHOLD)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.Turret.REVERSE_SOFT_LIMIT_THRESHOLD)
                .withReverseSoftLimitEnable(true);

        FeedbackConfigs turretFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Turret.GEAR_RATIO);

        yawMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

        TalonFXConfiguration yawConfig = new TalonFXConfiguration()
                .withMotionMagic(turretMotionMagicConfig)
                .withSlot0(turretSlot0Configs)
                .withMotorOutput(turretMotorOutputConfigs)
                .withCurrentLimits(turretCurrentLimitsConfigs)
                .withTorqueCurrent(turretTorqueCurrentConfigs)
                .withSoftwareLimitSwitch(turretSoftwareLimitSwitchConfigs)
                .withFeedback(turretFeedbackConfigs);

        // END TURRET CONFIG

        // START FLYWHEEL CONFIG

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

        CurrentLimitsConfigs shooterCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true);

        FeedbackConfigs shooterFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Shooter.FLYWHEEL_GEAR_RATIO);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                .withSlot0(shooterSlot0Configs)
                .withMotorOutput(shooterMotorOutputConfigs)
                .withTorqueCurrent(shooterTorqueCurrentConfigs)
                .withCurrentLimits(shooterCurrentLimits)
                .withFeedback(shooterFeedbackConfigs);

        yawMotor.getConfigurator().apply(yawConfig);
        shooterMotor.getConfigurator().apply(shooterConfig);
        shooterFollower.setControl(new Follower(Constants.Ports.SHOOTER_MOTOR, MotorAlignmentValue.Opposed));
        this.yawMotor.setControl(yawMotorControl.withPosition(0));

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

    Supplier<Pose2d> posSupplier = null;

    public void setPoseSupplier(Supplier<Pose2d> positionSupplier) {
        this.posSupplier = positionSupplier;
    }

    private boolean isManual = false;

    public Command manualAdjust(boolean left) {
        return this.runOnce(() -> {
            this.isManual = true;
            // this.shooterMotor.setControl(new VoltageOut(12.0));
            this.yawMotor.setControl(new VoltageOut(
                    left ? Constants.Turret.ADJUST_VOLTAGE : -Constants.Turret.ADJUST_VOLTAGE));
        }).andThen(this.run(() -> {
        })).finallyDo(() -> {
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
            SwerveDriveState robotState = chassisStateSupplier.get();
            Pose2d turretPose = robotState.Pose;

            Translation2d targetPose = AimAlign.getEffectivePose(turretPose, AimAlign.getZone(turretPose).getTranslation(), robotState.Speeds);
            this.shooterMotor.setControl(shooterSpeedControl.withVelocity(
                    AimAlign.getRequiredSpeed(turretPose, targetPose)));
            this.yawMotor.setControl(yawMotorControl.withPosition(
                    AimAlign.yawAngleToPos(turretPose, targetPose) / (2.0 * Math.PI)));
        }).withName("Continuously Aim Automatically");
    }

    public Command shoot(Supplier<SwerveDriveState> chassisStateSupplier) {
        return this.run(() -> {
            Pose2d turretPose = chassisStateSupplier.get().Pose;
            Translation2d targetPose = AimAlign.getZone(turretPose).getTranslation();
            this.shooterMotor.setControl(shooterSpeedControl.withVelocity(
                    AimAlign.getRequiredSpeed(turretPose, targetPose)));
        }).withName("Shoot without aiming");
    }

    public Command idle() {
        return this.runOnce(() -> {
            if (this.isManual)
                return;
            // point turret forward / starting orientation / whatever
            this.yawMotor.setControl(yawMotorControl.withPosition(0));
            this.shooterMotor.setControl(new VoltageOut(Constants.Shooter.IDLE_VOLTAGE));
        }).andThen(this.run(() -> {
        })).withName("Idle");
    }

    // make this default command
    public Command off() {
        return this.runOnce(() -> {
            if (this.isManual)
                return;
            this.shooterMotor.setControl(new CoastOut());
            this.yawMotor.setControl(yawMotorControl.withPosition(0));
        }).withName("Shooter Off");
    }

    public Command tuningSpeed(DoubleSupplier RPM) {
        return this.runOnce(() -> {
            this.yawMotor.setControl(yawMotorControl.withPosition(0));
            this.shooterFollower.setControl(
                    new Follower(Constants.Ports.SHOOTER_MOTOR, MotorAlignmentValue.Aligned));
            this.shooterMotor.setControl(shooterSpeedControl.withVelocity(RPM.getAsDouble() / 60.0));
        }).andThen(this.run(() -> {
        }));
    }

    private double targetYawRateOfChange = 0;
    private double oldTargetYaw = this.yawMotorControl.Position;
    private double timeSinceOldTargetYawUpdate;
    private Pose2d turretPose = new Pose2d();
    private final StructPublisher<Pose2d> turretPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("turretPos", Pose2d.struct).publish();

    @Override
    public void periodic() {
        if (posSupplier != null) {
            turretPose = posSupplier.get().plus(Constants.Turret.POSITION_TO_ROBOT);
            turretPose = turretPose
                    .plus(new Transform2d(new Translation2d(), new Rotation2d(this.yawMotor.getPosition().getValue())));
            SmartDashboard.putNumber("distance to target shot",
                    turretPose.getTranslation().minus(AimAlign.getZone(turretPose).getTranslation()).getNorm());
            SmartDashboard.putNumber("distance to blue hub",
                    turretPose.getTranslation().minus(FieldConstants.Hub.CENTER_POINT.getTranslation()).getNorm());
            SmartDashboard.putNumber("distance to red hub",
                    turretPose.getTranslation().minus(FieldConstants.Hub.RED_CENTER_POINT.getTranslation()).getNorm());
            turretPublisher.set(turretPose);
        }

        this.yawMotor.getPosition().refresh();
        this.shooterMotor.getVelocity().refresh();
        targetYawRateOfChange = (this.yawMotorControl.Position - this.oldTargetYaw)
                / (Utils.getCurrentTimeSeconds() - this.timeSinceOldTargetYawUpdate);
        oldTargetYaw = this.yawMotorControl.Position;
        timeSinceOldTargetYawUpdate = Utils.getCurrentTimeSeconds();

        // all these are in rotations per second
        SmartDashboard.putNumber("Yaw Angle", yawMotor.getPosition(false).getValueAsDouble());
        SmartDashboard.putNumber("Yaw Target Position", this.yawMotorControl.Position);
        SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getVelocity(false).getValueAsDouble());
        SmartDashboard.putNumber("Shooter Target Velocity", this.shooterSpeedControl.Velocity);
        SmartDashboard.putBoolean("Yaw Is At Position", this.yawIsAtPosition.getAsBoolean());
        SmartDashboard.putBoolean("Shooter Is At Velocity", this.shooterIsAtVelocity.getAsBoolean());
        SmartDashboard.putBoolean("Yaw Will Overturn Soon", this.yawWillOverturnSoon.getAsBoolean());
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
    private MechanismLigament2d yawArmLigament;

    private TalonFXSimState shooterMotorSimState;
    private final DCMotorSim shooterMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.0024, 1),
            DCMotor.getKrakenX60Foc(2));

    private final StructPublisher<Pose3d> yawPublisher = NetworkTableInstance.getDefault()
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

        yawMotorSimState.setRawRotorPosition(
                yawMotorSim.getAngleRads() * Constants.Turret.GEAR_RATIO / 2.0 / Math.PI);
        yawMotorSimState.setRotorVelocity(
                yawMotorSim.getVelocityRadPerSec() * Constants.Turret.GEAR_RATIO / 2.0 / Math.PI);

        shooterPos = new Pose3d(0, 0, 0,
                new Rotation3d(0, 0, yawMotor.getPosition().getValueAsDouble() * 2 * Math.PI));
        yawPublisher.set(shooterPos);

        shooterMotorSimState = shooterMotor.getSimState();
        double shooterSimVolts = shooterMotorSimState.getMotorVoltage();
        shooterMotorSim.setInputVoltage(shooterSimVolts);

        shooterMotorSim.update(deltatime);

        shooterMotorSimState.setRawRotorPosition(shooterMotorSim.getAngularPositionRotations() * Constants.Shooter.FLYWHEEL_GEAR_RATIO);
        shooterMotorSimState.setRotorVelocity(shooterMotorSim.getAngularVelocityRPM() / 60.0 * Constants.Shooter.FLYWHEEL_GEAR_RATIO);
        shooterMotorSimState.setRotorAcceleration(
                shooterMotorSim.getAngularAccelerationRadPerSecSq() * Constants.Shooter.FLYWHEEL_GEAR_RATIO / 2.0 / Math.PI);
    }

    public final Trigger yawIsAtPosition = new Trigger(
            () -> Util.epsilonEquals(this.yawMotor.getPosition(false).getValueAsDouble(),
                    this.yawMotorControl.Position, 0.01));

    public final Trigger shooterIsAtVelocity = new Trigger(
            () -> Util.epsilonEquals(this.shooterMotor.getVelocity(false).getValueAsDouble(),
                    this.shooterSpeedControl.Velocity, 5));

    public final Trigger yawWillOverturnSoon = new Trigger(() -> this.yawMotorControl.Position
            + this.targetYawRateOfChange
                    * Turret.OVERTURN_LOOKAHEAD_TIME > Constants.Turret.FORWARD_SOFT_LIMIT_THRESHOLD
            || this.yawMotorControl.Position + this.targetYawRateOfChange
                    * Turret.OVERTURN_LOOKAHEAD_TIME < Constants.Turret.REVERSE_SOFT_LIMIT_THRESHOLD);
}