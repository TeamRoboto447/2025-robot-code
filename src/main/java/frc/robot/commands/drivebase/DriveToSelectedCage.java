// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;

public class DriveToSelectedCage extends Command {
  private enum CageNum {
    ONE,
    TWO,
    THREE,
    DYNAMIC
  }

  private final CommandSwerveDrivetrain swerve;
  private Command driveCommand;
  private boolean done;
  private boolean oppositeSide;
  private SendableChooser<CageNum> targetCage;

  public DriveToSelectedCage(CommandSwerveDrivetrain swerve, boolean opposite) {
    this.swerve = swerve;
    this.oppositeSide = opposite;
    this.targetCage = new SendableChooser<CageNum>();
    this.targetCage.setDefaultOption("Driver Station Controlled", CageNum.DYNAMIC);
    this.targetCage.addOption("Left", CageNum.ONE);
    this.targetCage.addOption("Middle", CageNum.TWO);
    this.targetCage.addOption("Right", CageNum.THREE);
    SmartDashboard.putData("Target Climb Position", this.targetCage);
  }

  @Override
  public void initialize() {
    this.done = false;
    AllianceStationID allianceStationID = DriverStation.getRawAllianceStation();
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = Alliance.Blue;
    if (allianceOptional.isPresent())
      alliance = allianceOptional.get();

    final Pose2d cagePosition;
    CageNum targetCageNum = this.targetCage.getSelected();

    switch (targetCageNum) {
      case DYNAMIC:
        switch (allianceStationID) {
          case Blue1:
              cagePosition = this.oppositeSide ? FieldConstants.BlueSide.OPPOSITE_SIDE_CAGE_TWO : FieldConstants.BlueSide.CAGE_TWO;
          //   cagePosition = this.oppositeSide ? FieldConstants.BlueSide.OPPOSITE_SIDE_CAGE_ONE : FieldConstants.BlueSide.CAGE_ONE;
            break;
          case Blue2:
            cagePosition = this.oppositeSide ? FieldConstants.BlueSide.OPPOSITE_SIDE_CAGE_TWO : FieldConstants.BlueSide.CAGE_TWO;
            break;
          case Blue3:
            cagePosition = this.oppositeSide ? FieldConstants.BlueSide.OPPOSITE_SIDE_CAGE_TWO : FieldConstants.BlueSide.CAGE_TWO;
            // cagePosition = this.oppositeSide ? FieldConstants.BlueSide.OPPOSITE_SIDE_CAGE_THREE : FieldConstants.BlueSide.CAGE_THREE;
            break;
          case Red1:
            cagePosition = this.oppositeSide ? FieldConstants.RedSide.OPPOSITE_SIDE_CAGE_TWO : FieldConstants.RedSide.CAGE_TWO;
            // cagePosition = this.oppositeSide ? FieldConstants.RedSide.OPPOSITE_SIDE_CAGE_ONE : FieldConstants.RedSide.CAGE_ONE;
            break;
          case Red2:
            cagePosition = this.oppositeSide ? FieldConstants.RedSide.OPPOSITE_SIDE_CAGE_TWO : FieldConstants.RedSide.CAGE_TWO;
            break;
          case Red3:
            cagePosition = this.oppositeSide ? FieldConstants.RedSide.OPPOSITE_SIDE_CAGE_TWO : FieldConstants.RedSide.CAGE_TWO;
            // cagePosition = this.oppositeSide ? FieldConstants.RedSide.OPPOSITE_SIDE_CAGE_THREE : FieldConstants.RedSide.CAGE_THREE;
            break;
          default:
            cagePosition = null;
            break;
        }
        break;
      case ONE:
        if (alliance == Alliance.Blue)
          cagePosition = this.oppositeSide ? FieldConstants.BlueSide.OPPOSITE_SIDE_CAGE_ONE : FieldConstants.BlueSide.CAGE_ONE;
        else
          cagePosition = this.oppositeSide ? FieldConstants.RedSide.OPPOSITE_SIDE_CAGE_ONE : FieldConstants.RedSide.CAGE_ONE;
        break;
      case TWO:
        if (alliance == Alliance.Blue)
          cagePosition = this.oppositeSide ? FieldConstants.BlueSide.OPPOSITE_SIDE_CAGE_TWO : FieldConstants.BlueSide.CAGE_TWO;
        else
          cagePosition = this.oppositeSide ? FieldConstants.RedSide.OPPOSITE_SIDE_CAGE_TWO : FieldConstants.RedSide.CAGE_TWO;
        break;
      case THREE:
        if (alliance == Alliance.Blue)
        cagePosition = this.oppositeSide ? FieldConstants.BlueSide.OPPOSITE_SIDE_CAGE_THREE : FieldConstants.BlueSide.CAGE_THREE;
        else
        cagePosition = this.oppositeSide ? FieldConstants.RedSide.OPPOSITE_SIDE_CAGE_THREE : FieldConstants.RedSide.CAGE_THREE;
        break;
      default:
        cagePosition = null;
        break;

    }
    if (cagePosition != null) {
      PathConstraints constraints = new PathConstraints(
          Constants.SwerveSubsystemConstants.MaxCapableSpeed,
          Units.feetToMeters(5),
          2,
          Constants.SwerveSubsystemConstants.MaxAngularRate);

      // Create new command instance with latest pose
      driveCommand = AutoBuilder.pathfindToPose(
          cagePosition,
          constraints,
          edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
      )
          .finallyDo(interrupted -> {
            if (!interrupted) {
              System.out.println("Reached target: " + cagePosition);
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