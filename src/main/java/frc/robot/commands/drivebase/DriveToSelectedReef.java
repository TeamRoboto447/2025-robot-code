// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.controllers.ReefscapeStreamdeckController;
import frc.robot.controllers.StreamdeckController.ControlScheme;

public class DriveToSelectedReef extends Command {
  private final SwerveSubsystem swerve;
  private final ReefscapeStreamdeckController controller;
  private Command driveCommand;
  private boolean done;

  public DriveToSelectedReef(SwerveSubsystem swerve, ReefscapeStreamdeckController controller) {
    this.swerve = swerve;
    this.controller = controller;
  }

  @Override
  public void initialize() {
    this.done = false;
    // Get fresh pose data every initialization

    
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = Alliance.Blue;
    if (allianceOptional.isPresent())
      alliance = allianceOptional.get();
    Optional<Pose2d> poseOptional = controller.getTargetReefPosition(
        alliance,
        controller.leftSide.getAsBoolean());

      System.out.println(poseOptional);
    if (poseOptional.isPresent() && this.controller.getCurrentScheme() == ControlScheme.FULLYAUTO) {
      PathConstraints constraints = new PathConstraints(
          // swerve.getSwerveDrive().getMaximumChassisVelocity(),
           Units.feetToMeters(5),
           2,
          swerve.getSwerveDrive().getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

      // Create new command instance with latest pose
      driveCommand = AutoBuilder.pathfindToPose(
          poseOptional.get(),
          constraints,
          edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
      )
      .finallyDo(interrupted -> {
        if (!interrupted) {
          System.out.println("Reached target: " + poseOptional.get());
          this.done = true;
        }
      });

      driveCommand.schedule();
    } else {
      driveCommand = null;
    }
  }

  @Override
  public void execute() {
    // System.out.println(this.isFinished());
  }

  @Override
  public boolean isFinished() {
    return this.done;
  }

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.cancel();
    }
  }
}