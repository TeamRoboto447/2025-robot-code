// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.USE_VISION;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final SwerveSubsystem swerveSubsystem;
  private final PhotonRunnable leftCamera;
  private final PhotonRunnable rightCamera;

  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    if (USE_VISION) {
      this.leftCamera = new PhotonRunnable(new PhotonCamera("leftCamera"), null);
      this.rightCamera = new PhotonRunnable(new PhotonCamera("rightCamera"), null);
      this.setDefaultCommand(this.createNotifierCommand());
    } else {
      this.leftCamera = null;
      this.rightCamera = null;
    }
  }

  @Override
  public void periodic() {

  }

  private NotifierCommand createNotifierCommand() {
    return new NotifierCommand(() -> {
      leftCamera.run();
      rightCamera.run();
    }, 0.02);
  }

  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget target : estimation.targetsUsed) {
      Transform3d t3d = target.getBestCameraToTarget();
      double distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance)
        smallestDistance = distance;
    }

    double poseAmbiguityFactor = estimation.targetsUsed.size() != 1 ? 1
        : Math.max(1, estimation.targetsUsed.get(0).getPoseAmbiguity() + VisionConstants.POSE_AMBIGUITY_SHIFTER
            * VisionConstants.POSE_AMBIGUITY_MULTIPLIER);

    double confidenceMultiplier = Math.max(
        1,
        (Math.max(
            1,
            Math.max(0, smallestDistance - VisionConstants.NOISY_DISTANCE_METERS)
                * VisionConstants.DISTANCE_WEIGHT)
            * poseAmbiguityFactor)
            / (1
                + ((estimation.targetsUsed.size() - 1)
                    * VisionConstants.TAG_PRESENCE_WEIGHT)));
    return VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    
  }

  public void estimatorChecker(PhotonRunnable estimator) {
    EstimatedRobotPose cameraPose = estimator.grabLatestEstimatedPose();
    if (cameraPose == null)
      return;
    Pose2d pose2d = cameraPose.estimatedPose.toPose2d();
    if (RobotState.isDisabled()) {
      swerveSubsystem.getSwerveDrive().resetOdometry(pose2d);
    } else {
      swerveSubsystem.getSwerveDrive().addVisionMeasurement(pose2d, cameraPose.timestampSeconds, confidenceCalculator);
    }
  }
}
