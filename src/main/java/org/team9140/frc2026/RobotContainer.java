// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2026;

import org.team9140.frc2026.commands.AutonomousRoutines;
import org.team9140.frc2026.generated.TunerConstants;
import org.team9140.frc2026.subsystems.Climber;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Hopper;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Climber climber = Climber.getInstance();
  private final Hopper hopper = Hopper.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();

  private final CommandXboxController controller = new CommandXboxController(0);
  private final SwerveTelemetry logger = new SwerveTelemetry(drivetrain, Constants.Drive.MAX_TELEOP_VELOCITY);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    this.controller.rightBumper().whileTrue(this.intake.intake());
    this.controller.leftBumper().whileTrue(this.intake.reverse().alongWith(this.hopper.unjam()));
    this.controller.a().onTrue(this.shooter.aim(() -> this.drivetrain.getState()));
    this.controller.x().onTrue(this.shooter.idle());
    this.controller.rightTrigger().onTrue(this.hopper.feed()).onFalse(this.hopper.off());
    this.controller.back().whileTrue(this.shooter.manualLeft());
    this.controller.start().whileTrue(this.shooter.manualRight());
    this.controller.y().onTrue(this.climber.extend());
    this.controller.b().onTrue(this.climber.retract());

    drivetrain.setDefaultCommand(
        drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY,
            controller::getRightX).ignoringDisable(true));

    this.drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return AutonomousRoutines.getInstance(drivetrain).getAutoChooser().getSelected();
  }
}