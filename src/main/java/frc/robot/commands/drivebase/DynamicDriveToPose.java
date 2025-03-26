// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.controllers.ReefscapeStreamdeckController;

public class DynamicDriveToPose extends Command {
  private final SwerveSubsystem swerve;
  private final ReefscapeStreamdeckController controller;
  private final Alliance alliance;
  private Command driveCommand;

  public DynamicDriveToPose(SwerveSubsystem swerve, ReefscapeStreamdeckController controller, Alliance alliance) {
    this.swerve = swerve;
    this.controller = controller;
    this.alliance = alliance;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Get fresh pose data every initialization

    Optional<Pose2d> poseOptional = controller.getTargetReefPosition(
        alliance,
        controller.leftSide.getAsBoolean());

    if (poseOptional.isPresent()) {
      // Create new command instance with latest pose
      // driveCommand = swerve.driveToPose(poseOptional.get())
      // .finallyDo(interrupted -> {
      // if (!interrupted) {
      // System.out.println("Reached target: " + poseOptional.get());
      // }
      // });
      driveCommand = Commands.runOnce(() -> System.out.println(poseOptional.get().toString()));

      driveCommand.schedule();
    } else {
      driveCommand = null;
    }
  }

  @Override
  public boolean isFinished() {
    return driveCommand == null || driveCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.cancel();
    }
  }
}