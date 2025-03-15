// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.USE_PHOTON_VISION;
import static frc.robot.Constants.VisionConstants.USE_QUEST_NAV;

import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_FRONT_CAM;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PhotonCamera frontCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    frontCamera = new PhotonCamera("FrontCam");
    poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_FRONT_CAM);
  }

  @Override
  public void periodic() {
    if (!USE_PHOTON_VISION)
      return;
    List<PhotonPipelineResult> pipeline = frontCamera.getAllUnreadResults();
    for (PhotonPipelineResult result : pipeline) {
      Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
      if (estimatedPose.isEmpty())
        continue;
      Matrix<N3, N1> standardDeviations = calculateStandardDeviations(estimatedPose.get());
      if (RobotState.isDisabled())
        swerveSubsystem.resetOdometry(estimatedPose.get().estimatedPose);

      if (!USE_QUEST_NAV) {
        Pose2d poseEstimate = estimatedPose.get().estimatedPose.toPose2d();
        double error = poseEstimate.getTranslation().getDistance(swerveSubsystem.getPose().getTranslation());
        // TODO: Armstrong added this on 3/15/2025 around 1:38 p.m.
        if (error < 0.25) {
          swerveSubsystem.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp(),
              standardDeviations);
        }
      }
    }
  }

  public Pose2d getCurrentPose() {
    return swerveSubsystem.getPose();
  }

  public void setCurrentPose(Pose2d newPose) {
    swerveSubsystem.getSwerveDrive().resetOdometry(newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  private Matrix<N3, N1> calculateStandardDeviations(EstimatedRobotPose pose) {
    int numTargets = pose.targetsUsed.size();

    double baseXYStdDev = 0.5; // meters
    double baseThetaStdDev = 0.1; // radians

    if (numTargets > 1) {
      return VecBuilder.fill(baseXYStdDev / 2, baseXYStdDev / 2, baseThetaStdDev / 2);
    } else if (numTargets == 1) {
      PhotonTrackedTarget target = pose.targetsUsed.get(0);
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      double factor = 1.0 + (distance / 4.0);
      return VecBuilder.fill(baseXYStdDev * factor, baseXYStdDev * factor, baseThetaStdDev * factor);
    } else {
      return VecBuilder.fill(5.0, 5.0, 10.0); // High uncertainty when no targets are reliably detected
    }
  }

}
