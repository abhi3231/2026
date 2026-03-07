package org.team9140.frc2026.commands;

import java.util.Optional;

import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.subsystems.Climber;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Hopper;
import org.team9140.frc2026.subsystems.Shooter;
import org.team9140.lib.FollowPath;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousRoutines {
    private final CommandSwerveDrivetrain drivetrain;

    private final Shooter shooter = Shooter.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Hopper hopper = Hopper.getInstance();

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    private static AutonomousRoutines instance;

    public static AutonomousRoutines getInstance(CommandSwerveDrivetrain drivetrain) {
        return (instance == null) ? instance = new AutonomousRoutines(drivetrain) : instance;
    }

    private AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        autoChooser.setDefaultOption("Do Nothing", "nothing");
        autoChooser.addOption("Shoot Preload", "preload");
        autoChooser.addOption("Climb Left", "climb_left");
        autoChooser.addOption("Climb Right", "climb_right");
        autoChooser.addOption("Sweep Middle From Left", "sweep_middle_left");
        autoChooser.addOption("Sweep Middle From Right", "sweep_middle_right");
        SmartDashboard.putData(autoChooser);
    }

    public Command getCommand() {
        switch (autoChooser.getSelected()) {
            case "preload":
                return shootPreload(3);
            case "climb_left":
                return climb(true);
            case "climb_right":
                return climb(false);
            case "sweep_middle_left":
                return sweepMiddleFromLeft();
            case "sweep_middle_right":
                return sweepMiddleFromRight();
            default:
                return doNothing();
        }
    }

    public Command doNothing() {
        return new PrintCommand("Doing Nothing");
    }

    public Command shootPreload(double seconds) {
        return Commands.deadline(
            new WaitCommand(seconds),
            shooter.aim(
                () -> this.drivetrain.getState()
            ),
            hopper.feed() 
        );
    }

    public Command climb(boolean left) {
        Pose2d goalPos;
        if(left) {
            if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
                goalPos = FieldConstants.Tower.LEFT_UPRIGHT;
            } else {
                goalPos = FieldConstants.Tower.RED_LEFT_UPRIGHT;
            }
        } else {
            if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
                goalPos = FieldConstants.Tower.RIGHT_UPRIGHT;
            } else {
                goalPos = FieldConstants.Tower.RED_RIGHT_UPRIGHT;
            }
        }
        return this.drivetrain.goToPose(() -> goalPos)
            .until(this.drivetrain.reachedPose)
            .andThen(climber.extend()).andThen(climber.retract());
    }

    public Command sweepMiddleFromRight() {
        FollowPath path = new FollowPath("crossandsweep_Blue_Right", () -> this.drivetrain.getState().Pose,
                this.drivetrain::followSample, Util.getAlliance().get(), drivetrain);
        return path.gimmeCommand();
    }

    public Command sweepMiddleFromLeft() {
        FollowPath path = new FollowPath("crossandsweep_Blue_Left", () -> this.drivetrain.getState().Pose,
                this.drivetrain::followSample, Util.getAlliance().get(), drivetrain);
        return path.gimmeCommand();
    }
}