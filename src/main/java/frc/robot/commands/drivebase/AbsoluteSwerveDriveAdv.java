// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AbsoluteSwerveDriveAdv extends Command {
  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingAdjust;
  private final BooleanSupplier lookBack, lookForwards, lookLeft, lookRight;
  private boolean resetHeading = false;
  /** Creates a new AbsoluteDriveAdv. */
  public AbsoluteSwerveDriveAdv(
    SwerveSubsystem swerveSubsystem, DoubleSupplier vX, DoubleSupplier vY,
    DoubleSupplier headingAdjust, BooleanSupplier lookBack, BooleanSupplier lookForwards,
    BooleanSupplier lookLeft, BooleanSupplier lookRight) {
    this.swerve = swerveSubsystem;
    this.vX = vX;
    this.vY = vY;
    this.headingAdjust = headingAdjust;
    this.lookBack = lookBack;
    this.lookForwards = lookForwards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;
    
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetHeading = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double headingX = 0;
    double headingY = 0;

    if (lookBack.getAsBoolean()) {
      headingY = -1;
    }
    if (lookRight.getAsBoolean()) {
      headingX = 1;
    }
    if (lookLeft.getAsBoolean()) {
      headingX = -1;
    }
    if (lookForwards.getAsBoolean()) {
      headingY = 1;
    }

    if (resetHeading) {
      if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) == 0) {
        Rotation2d currentHeading = swerve.getHeading();

        headingX = currentHeading.getSin();
        headingY = currentHeading.getCos();
      }
      resetHeading = false;
    }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
    Constants.SwerveSubsystemConstants.LOOP_TIME, Constants.SwerveSubsystemConstants.ROBOT_MASS, List.of(Constants.SwerveSubsystemConstants.CHASSIS),
    swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());
    if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0)
    {
       resetHeading = true;
       swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble()), true);
    } else
    {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
