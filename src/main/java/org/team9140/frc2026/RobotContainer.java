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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final AutonomousRoutines autoRoutines;

  private final Vision limeA = new Vision(Constants.Vision.CAMERA_NAMES[0], this.drivetrain::acceptVisionMeasurement, Constants.Vision.ROBOT_TO_CAM[0]);
  private final Vision limeB = new Vision(Constants.Vision.CAMERA_NAMES[1], this.drivetrain::acceptVisionMeasurement, Constants.Vision.ROBOT_TO_CAM[1]);

  public RobotContainer() {
    limeA.setIMUMode(1);
    limeB.setIMUMode(1);

    configureBindings();
    autoRoutines = AutonomousRoutines.getInstance(drivetrain);
  }

  private void configureBindings() {
    SmartDashboard.putNumber("RPM", 2500);
    this.controller.rightBumper().onTrue(this.intake.intake()).onFalse(this.intake.off());
    this.controller.a().onTrue(this.intake.reverse().alongWith(this.hopper.unjam()))
        .onFalse(this.intake.off().alongWith(this.hopper.off()));
    this.controller.a().whileTrue(this.shooter.tuningSpeed(() -> SmartDashboard.getNumber("RPM", 3500)));
    this.controller.x().onTrue(this.shooter.idle());
    this.controller.rightTrigger().whileTrue(this.hopper.feed());
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
    return autoRoutines.getCommand();
  }
}