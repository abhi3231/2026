package org.team9140.lib;

import static org.team9140.lib.Util.rotationEpsilonEquals;

import java.util.Optional;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// TODO: Support path splits
public class FollowPath {

    private final EventLoop loop;
    private final Timer timer;
    private final DriverStation.Alliance alliance;
    private final Trigger activeTrigger;
    private final TreeMap<String, Trigger> eventTimes;

    private Trajectory<SwerveSample> trajectory;
    private boolean active = false;

    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<SwerveSample> sampleConsumer;

    private final Subsystem requirement;

    private final Pose2d finalPose;

    public FollowPath(String name, Supplier<Pose2d> poseSupplier, Consumer<SwerveSample> sampleConsumer,
            DriverStation.Alliance alliance, Subsystem requirement) {
        Choreo.<SwerveSample>loadTrajectory(name)
                .ifPresent(trajectory -> this.trajectory = alliance.equals(DriverStation.Alliance.Blue) ? trajectory
                        : trajectory.flipped());

        this.loop = new EventLoop();
        this.timer = new Timer();
        this.alliance = alliance;
        this.eventTimes = new TreeMap<>();

        this.activeTrigger = new Trigger(loop, () -> this.active);

        this.poseSupplier = poseSupplier;
        this.sampleConsumer = sampleConsumer;

        this.requirement = requirement;

        this.finalPose = this.trajectory.getFinalPose(false).get();
        for (EventMarker e : this.trajectory.events()) {
            this.eventTimes.put(e.event, atTime(e.timestamp));
            System.out.println("Added event " + e.event + " at time " + e.timestamp);

        }
    }

    public TreeMap<String, Trigger> getEvents() {
        return eventTimes;
    }

    public Trigger atEventTime(String eventName) {
        return this.eventTimes.get(eventName);
    }

    public Trigger removeEvent(String eventName) {
        return this.eventTimes.remove(eventName);
    }
    public Trigger atTime(double timestamp) {
        return new Trigger(
                loop,
                () -> timer.get() >= timestamp)
                .and(activeTrigger);
    }

    public Trigger atPose(Pose2d pose, double toleranceMeters, double toleranceRadians) {
        Pose2d flippedPose = ChoreoAllianceFlipUtil.flip(pose);

        if (this.alliance.equals(DriverStation.Alliance.Red)) {
            return positionTrigger(flippedPose, toleranceMeters, toleranceRadians);
        }
        return positionTrigger(pose, toleranceMeters, toleranceRadians);
    }

    private Trigger positionTrigger(Pose2d pose, double toleranceMeters, double toleranceRadians) {
        return new Trigger(
                loop,
                () -> {
                    final Pose2d currentPose = this.poseSupplier.get();
                    boolean transValid = currentPose.getTranslation()
                            .getDistance(pose.getTranslation()) < toleranceMeters;
                    boolean rotValid = rotationEpsilonEquals(
                            currentPose.getRotation(), pose.getRotation(), toleranceRadians);
                    return transValid && rotValid;
                })
                .and(activeTrigger);
    }

    public Trigger atTranslation(Translation2d translation, double toleranceMeters) {
        Translation2d flippedTranslation = ChoreoAllianceFlipUtil.flip(translation);

        if (this.alliance.equals(DriverStation.Alliance.Red)) {
            return new Trigger(
                    loop,
                    () -> {
                        final Translation2d currentTrans = this.poseSupplier.get().getTranslation();
                        return currentTrans.getDistance(flippedTranslation) < toleranceMeters;
                    })
                    .and(activeTrigger);
        }

        return new Trigger(
                loop,
                () -> {
                    final Translation2d currentTrans = this.poseSupplier.get().getTranslation();
                    return currentTrans.getDistance(translation) < toleranceMeters;
                })
                .and(activeTrigger);
    }

    public Pose2d getInitialPose() {
        return this.trajectory.getInitialPose(false).get();
    }

    public Pose2d getFinalPose() {
        return this.finalPose;
    }

    public Command gimmeCommand() {
        return new FunctionalCommand(
                () -> {
                    this.timer.restart();
                    this.active = true;
                },
                () -> {
                    this.loop.poll();
                    Optional<SwerveSample> sample = this.trajectory.sampleAt(this.timer.get(), false);
                    sample.ifPresent(this.sampleConsumer);
                },
                interrupted -> {
                    this.active = false;
                },
                () -> (this.timer.hasElapsed(this.trajectory.getTotalTime()))
                        && Util.epsilonEquals(poseSupplier.get(), getFinalPose()),
                requirement);
    }
}