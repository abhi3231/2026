package org.team9140.frc2026.commands;

import java.util.Optional;

import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.Robot;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Hopper;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;
import org.team9140.lib.FollowPath;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class AutonomousRoutines {
    private final CommandSwerveDrivetrain drivetrain;

    private final Shooter shooter = Shooter.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Intake intake = Intake.getInstance();

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private static AutonomousRoutines instance;

    public static AutonomousRoutines getInstance(CommandSwerveDrivetrain drivetrain) {
        return (instance == null) ? instance = new AutonomousRoutines(drivetrain) : instance;
    }

    private AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        autoChooser.setDefaultOption("Do Nothing", "nothing");
        autoChooser.addOption("Shoot Preload", "preload");
        autoChooser.addOption("Score from Depot starting Middle", "score_from_depot");
        autoChooser.addOption("Score from Depot starting Left", "depot_shoot_left");
        autoChooser.addOption("1 Pass From Depot", "one_pass_depot");
        autoChooser.addOption("1 Pass From Depot then Score from Depot", "one_pass_depot_then_depot_shoot");
        autoChooser.addOption("1 Pass From Outpost", "one_pass_outpost");
        autoChooser.addOption("2 Passes from Outpost", "two_pass_outpost");
        autoChooser.addOption("2 Passes from Depot", "two_pass_depot");
        autoChooser.addOption("2 Safe Passes from Outpost", "two_pass_outpost_safe");
        autoChooser.addOption("2 Safe Passes from Depot", "two_pass_depot_safe");

        autoChooser.addOption("1 Pass Over Bump From Depot", "one_pass_depot_bump");
        autoChooser.addOption("1 Pass Over Bump From Outpost", "one_pass_outpost_bump");
        autoChooser.addOption("2 Passes Over Bump from Outpost", "two_pass_outpost_bump");
        autoChooser.addOption("2 Passes Over Bump from Depot", "two_pass_depot_bump");
        autoChooser.addOption("2 Safe Passes Over Bump from Outpost", "two_pass_outpost_safe_bump");
        autoChooser.addOption("2 Safe Passes Over Bump from Depot", "two_pass_depot_safe_bump");
        SmartDashboard.putData(autoChooser);
    }

    private Command getShootCommand() {
        return shooter.aim(this.drivetrain::getCachedState)
                .alongWith(new WaitUntilCommand(shooter.readyToShoot)
                        .andThen(hopper.feed()));
    }

    private DriverStation.Alliance lastAlliance = Alliance.Red;
    private String lastFetchedAuto = "";

    private Translation2d targetHub = FieldConstants.Hub.CENTER_POINT.getTranslation();

    public Command getCommand() {
        if (Util.getAlliance().isPresent() && (!Util.getAlliance().get().equals(lastAlliance)
                || (lastFetchedAuto != (lastFetchedAuto = autoChooser.getSelected())))) {
            lastAlliance = Util.getAlliance().get();
            switch (lastFetchedAuto) {
                case "preload":
                    if (Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Red))) {
                        targetHub = FieldConstants.Hub.RED_CENTER_POINT.getTranslation();
                    } else {
                        targetHub = FieldConstants.Hub.CENTER_POINT.getTranslation();
                    }
                    return this.intake.armOut().andThen(new WaitCommand(2.0)).andThen(shooter.aim(this.drivetrain::getCachedState, () -> targetHub));
                case "depot_shoot_left":
                    return intake.intake().alongWith(runChoreoAuto("depotShoot_Left").andThen(this.getShootCommand().asProxy()));
                case "score_from_depot":
                    return intake.intake().alongWith(runChoreoAuto("depotShoot").andThen(this.getShootCommand().asProxy()));
                case "one_pass_outpost":
                    return intake.intake().alongWith(runChoreoAuto("Trench_Outpost_Deep").andThen(this.getShootCommand().asProxy()));
                case "one_pass_depot":
                    return intake.intake().alongWith(runChoreoAuto("Trench_Depot_Deep").andThen(this.getShootCommand().asProxy()));
                case "two_pass_outpost":
                    return intake.intake().alongWith(runChoreoAuto("Trench_Outpost_Deep", false, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("Trench_Reset_Outpost", false, false))
                    .andThen(runChoreoAuto("Trench_Outpost_Shallow", true, false))
                    .andThen(this.getShootCommand().asProxy()));
                case "two_pass_depot":
                    return intake.intake().alongWith(runChoreoAuto("Trench_Depot_Deep", false, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("Trench_Reset_Depot", false, false))
                    .andThen(runChoreoAuto("Trench_Depot_Shallow", true, false))
                    .andThen(this.getShootCommand().withTimeout(5).asProxy()));
                case "two_pass_outpost_safe":
                    return intake.intake().alongWith(runChoreoAuto("Trench_Outpost_Shallow", false, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("Trench_Reset_Outpost", false, false))
                    .andThen(runChoreoAuto("Trench_Outpost_Shallow", true, false))
                    .andThen(this.getShootCommand().asProxy()));
                case "two_pass_depot_safe":
                    return intake.intake().alongWith(runChoreoAuto("Trench_Depot_Shallow", false, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("Trench_Reset_Depot", false, false))
                    .andThen(runChoreoAuto("Trench_Depot_Shallow", true, false))
                    .andThen(this.getShootCommand().withTimeout(5).asProxy()));
                case "one_pass_outpost_bump":
                    return intake.intake().alongWith(runChoreoAuto("Bump_Outpost_Deep").andThen(this.getShootCommand().asProxy()));
                case "one_pass_depot_bump":
                    return intake.intake().alongWith(runChoreoAuto("Bump_Depot_Deep").andThen(this.getShootCommand().asProxy()));
                case "two_pass_outpost_bump":
                    return intake.intake().alongWith(runChoreoAuto("Bump_Outpost_Deep", true, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("Bump_Reset_Outpost", false, false))
                    .andThen(runChoreoAuto("Bump_Outpost_Shallow", true, false))
                    .andThen(this.getShootCommand().asProxy()));
                case "two_pass_depot_bump":
                    return intake.intake().alongWith(runChoreoAuto("Bump_Depot_Deep", false, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("Bump_Reset_Depot", false, false))
                    .andThen(runChoreoAuto("Bump_Depot_Shallow", true, false))
                    .andThen(this.getShootCommand().asProxy()));
                case "two_pass_outpost_safe_bump":
                    return intake.intake().alongWith(runChoreoAuto("Bump_Outpost_Shallow", false, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("Bump_Reset_Outpost", false, false))
                    .andThen(runChoreoAuto("Bump_Outpost_Shallow", true, false))
                    .andThen(this.getShootCommand().asProxy()));
                case "two_pass_depot_safe_bump":
                    return intake.intake().alongWith(runChoreoAuto("Bump_Depot_Shallow", false, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("Bump_Reset_Depot", false, false))
                    .andThen(runChoreoAuto("Bump_Depot_Shallow", true, false))
                    .andThen(this.getShootCommand().asProxy()));
                case "one_pass_depot_then_depot_shoot":
                    return intake.intake().alongWith(runChoreoAuto("Trench_Depot_Deep", false, true)
                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                    .andThen(runChoreoAuto("depotShoot_Left", true, false))
                    .andThen(this.getShootCommand().asProxy()));
                default:
                    return doNothing();
            }
        }

        return null;
    }

    public Command doNothing() {
        return new PrintCommand("Doing Nothing");
    }


    private final StructPublisher<Pose2d> initialPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Auto Path Initial Pose", Pose2d.struct).publish();

    public Command runChoreoAuto(String pathame, boolean waitUntilAtFinalTarget, boolean reset) {
        FollowPath path = new FollowPath(pathame, () -> this.drivetrain.getState().Pose,
                this.drivetrain::followSample, Util.getAlliance().get(), drivetrain);
        if (Robot.isSimulation() && reset)
            drivetrain.resetPose(path.getInitialPose());
        initialPosePublisher.set(path.getInitialPose());
        return path.gimmeCommand(waitUntilAtFinalTarget);
    }

    public Command runChoreoAuto(String pathame) {
        return this.runChoreoAuto(pathame, true, true);
    }

}