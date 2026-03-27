package org.team9140.frc2026.commands;

import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.function.Supplier;

import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.Robot;
import org.team9140.frc2026.subsystems.Climber;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Hopper;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;
import org.team9140.lib.FollowPath;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutonomousRoutines {
    private final CommandSwerveDrivetrain drivetrain;

    private final Shooter shooter = Shooter.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Intake intake = Intake.getInstance();

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final HashMap<String, Supplier<Command>> namedCommands = new HashMap<>();
    private static AutonomousRoutines instance;

    public static AutonomousRoutines getInstance(CommandSwerveDrivetrain drivetrain) {
        return (instance == null) ? instance = new AutonomousRoutines(drivetrain) : instance;
    }

    private AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        autoChooser.setDefaultOption("Do Nothing", "nothing");
        autoChooser.addOption("Shoot Preload", "preload");
        autoChooser.addOption("Sweep Middle From Depot", "sweep_middle_depot");
        autoChooser.addOption("Sweep Middle From Outpost", "sweep_middle_outpost");
        autoChooser.addOption("Score from Depot starting Middle", "score_from_depot");
        autoChooser.addOption("1 Pass From Depot", "one_pass_depot");
        autoChooser.addOption("1 Pass From Outpost", "one_pass_outpost");
        autoChooser.addOption("2 Passes from Outpost", "two_pass_outpost");
        autoChooser.addOption("2 Passes from Depot", "two_pass_depot");
        autoChooser.addOption("2 Safe Passes from Outpost", "two_pass_outpost_safe");
        autoChooser.addOption("2 Safe Passes from Depot", "two_pass_depot_safe");
        SmartDashboard.putData(autoChooser);
        namedCommands.put("shoot", this::getShootCommand);
        namedCommands.put("intakeOn", intake::intake);
        namedCommands.put("intakeOff", intake::off);
        namedCommands.put("shootOff", this::getShootOffCommand);
    }

    private Command getShootCommand() {
        return shooter.aim(this.drivetrain::getCachedState)
                .alongWith(new WaitUntilCommand(shooter.yawIsAtPosition.and(shooter.shooterIsAtVelocity))
                        .andThen(hopper.feed()));
    }

    private Command getShootOffCommand() {
        return shooter.off().andThen(hopper.off());
    }

    private DriverStation.Alliance lastAlliance = Alliance.Red;
    private String lastFetchedAuto = "";

    public Command getCommand() {
        if (Util.getAlliance().isPresent() && (!Util.getAlliance().get().equals(lastAlliance)
                || (lastFetchedAuto != (lastFetchedAuto = autoChooser.getSelected())))) {
            lastAlliance = Util.getAlliance().get();
            switch (lastFetchedAuto) {
                case "preload":
                    return this.intake.armOut().andThen(new WaitCommand(2.0)).andThen(this.getShootCommand());
                case "sweep_middle_depot":
                    return runChoreoAuto("crossandsweep_Depot");
                case "sweep_middle_outpost":
                    return runChoreoAuto("crossandsweep_Outpost");
                case "score_from_depot":
                    return runChoreoAuto("depotShoot");
                case "one_pass_outpost":
                    return runChoreoAuto("onceReverse_Outpost");
                case "one_pass_depot":
                    return runChoreoAuto("onceReverse_Depot");
                case "two_pass_outpost":
                    return runChoreoAuto("repeatReverse_Outpost_Deep", false)
                    .andThen(runChoreoAuto("repeatReverse_Outpost_Shallow"));
                case "two_pass_depot":
                    return runChoreoAuto("repeatReverse_Depot_Deep", false)
                    .andThen(runChoreoAuto("repeatReverse_Depot_Shallow"));
                case "two_pass_outpost_safe":
                    return runChoreoAuto("repeatReverse_Outpost_Deep_SAFE", false)
                    .andThen(runChoreoAuto("repeatReverse_Outpost_Shallow"));
                case "two_pass_depot_safe":
                    return runChoreoAuto("repeatReverse_Depot_Deep_SAFE", false)
                    .andThen(runChoreoAuto("repeatReverse_Depot_Shallow"));
                default:
                    return doNothing();
            }
        }

        return null;
    }

    public Command doNothing() {
        return new PrintCommand("Doing Nothing");
    }

    public Command climb(boolean left) {
        Pose2d goalPos;
        if (left) {
            if (Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
                goalPos = FieldConstants.Tower.LEFT_UPRIGHT;
            } else {
                goalPos = FieldConstants.Tower.RED_LEFT_UPRIGHT;
            }
        } else {
            if (Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
                goalPos = FieldConstants.Tower.RIGHT_UPRIGHT;
            } else {
                goalPos = FieldConstants.Tower.RED_RIGHT_UPRIGHT;
            }
        }
        return this.drivetrain.goToPose(() -> goalPos)
                .until(this.drivetrain.reachedPose)
                .andThen(climber.extend()).andThen(climber.retract());
    }

    public void bindEventCommands(FollowPath path) {
        for (Entry<String, Trigger> entry : path.getEvents().entrySet()) {
            entry.getValue().onTrue(namedCommands.get(entry.getKey()).get());
        }
    }

    private final StructPublisher<Pose2d> initialPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Auto Path Initial Pose", Pose2d.struct).publish();

    public Command runChoreoAuto(String pathame, boolean waitUntilAtFinalTarget) {
        FollowPath path = new FollowPath(pathame, () -> this.drivetrain.getState().Pose,
                this.drivetrain::followSample, Util.getAlliance().get(), drivetrain);
        if (Robot.isSimulation())
            drivetrain.resetPose(path.getInitialPose());
        this.bindEventCommands(path);
        initialPosePublisher.set(path.getInitialPose());
        return path.gimmeCommand(waitUntilAtFinalTarget);
    }

    public Command runChoreoAuto(String pathame) {
        return this.runChoreoAuto(pathame, true);
    }

}