// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class SwerveSubsystemConstants {
    public static final double MAX_SPEED = Units.feetToMeters(9.2); // 12.78 is max speed the robot is capable of
    public static final double ROBOT_MASS = Units.lbsToKilograms(107);
    public static final Matter CHASSIS = new Matter(new Translation3d(0.0, 0.0, Units.inchesToMeters(8)), ROBOT_MASS); // TODO:
                                                                                                                       // update
                                                                                                                       // with
                                                                                                                       // actual
                                                                                                                       // robot
                                                                                                                       // COG
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  }

  public static class ExampleSubsystemConstants {
    public static final int EXAMPLE_MOTOR_ID = 0; // This isn't a valid CAN ID, in your real subsystems make sure this
                                                  // matches the ID programmed into the
    public static final int EXAMPLE_LOWER_LIMIT_SWITCH_CHANNEL = 0;
    public static final int EXAMPLE_UPPER_LIMIT_SWITCH_CHANNEL = 1;
  }

  public static class DriverConstants {
    // Default generic deadband value
    public static final double DEADBAND = 0.1;

    // While these feel redundant, it will allow for individual tuning of each axis
    // in the future if necessary
    public static final double LEFT_X_DEADBAND = DEADBAND;
    public static final double LEFT_Y_DEADBAND = DEADBAND;
    public static final double RIGHT_X_DEADBAND = DEADBAND;
    public static final double RIGHT_Y_DEADBAND = DEADBAND;
    public static final double TURN_CONSTANT = 6;
  }

  public static class OperatorConstants {
    // Default generic deadband value
    public static final double DEADBAND = 0.1;

    // While these feel redundant, it will allow for individual tuning of each axis
    // in the future if necessary
    public static final double LEFT_X_DEADBAND = DEADBAND;
    public static final double LEFT_Y_DEADBAND = DEADBAND;
    public static final double RIGHT_X_DEADBAND = DEADBAND;
    public static final double RIGHT_Y_DEADBAND = DEADBAND;
  }

  public static class ClimberSubsystemConstants {
    public static final int CLIMBER_MAX_SPEED = 10;
    public static final int CLIMBER_MOTOR_ID = 20;
    public static final int CLIMBER_LOWER_LIMIT_ID = 9;
  }

  public static class ElevatorSubsystemConstants {
    public static final int ELEVATOR_MOTOR_ID = 22;
    public static final int AUXILLARY_ELEVATOR_MOTOR_ID = 23;
    public static final int ELEVATOR_UPPER_LIMIT_SWITCH_CHANNEL = 2;
    public static final int ELEVATOR_LOWER_LIMIT_SWITCH_CHANNEL = 3;

    public enum Level {
      FLOOR,
      FLOOR_COLLECT,
      CORAL_LOADING,
      TROUGH,
      CORAL_L2,
      CORAL_L3,
      CORAL_L4,
      ALGAE_L1,
      ALGAE_L2,
      NET
    }

    // public static final double MIN_INCH_HEIGHT = 11.5;
    // public static final double MAX_INCH_HEIGHT = 92;
    public static final int MIN_RAW_HEIGHT = 0;
    public static final int MAX_RAW_HEIGHT = 90;
    public static final double GEARING_MULTIPLIER = 4.5; // 4:1 gearbox
    public static final Distance DISTANCE_PER_ROTATION = Inches.of(1 * Math.PI); // Circumferance of 1 inch pulley

    public static final Distance FLOOR_LEVEL = Inches.of(0);
    public static final Distance FLOOR_COLLECT_LEVEL = Inches.of(9);
    public static final Distance CORAL_LOADING_LEVEL = Inches.of(6);
    public static final Distance TROUGH_LEVEL = FLOOR_LEVEL;
    public static final Distance CORAL_L2_LEVEL = FLOOR_LEVEL;
    public static final Distance CORAL_L3_LEVEL = Inches.of(16);

    public static final Distance ALGAE_L1_LEVEL = Inches.of(23);
    public static final Distance ALGAE_L2_LEVEL = Inches.of(33);

    public static final Distance CORAL_L4_LEVEL = Inches.of(46);
    public static final Distance NET_LEVEL = Inches.of(62);
  }

  public static class AlgaeManipulatorSubsystemConstants {
    public static final int UPPER_WHEEL_MOTOR_ID = 24;
    public static final int LOWER_WHEEL_MOTOR_ID = 25;
    public static final int WRIST_MOTOR_ID = 26;
    public static final int WRIST_MOTOR_PDH_CHANNEL = 17;
    public static final int CORAL_MOTOR_ID = 27;
  }

  public static class VisionConstants {
    public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(
        new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(10.125), Units.inchesToMeters(7.25)),
        new Rotation3d(0, Units.degreesToRadians(-24.5), 0));

    public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
        Units.inchesToMeters(-12),
        Units.inchesToMeters(-6.5),
        Units.inchesToMeters(9),
        new Rotation3d(0, 0, -130.7));

    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final boolean USE_PHOTON_VISION = false;
    public static final double POSE_AMBIGUITY_SHIFTER = 0;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 0;
    public static final double NOISY_DISTANCE_METERS = 0;
    public static final double DISTANCE_WEIGHT = 0;
    public static final int TAG_PRESENCE_WEIGHT = 0;

    public static final boolean USE_QUEST_NAV = true;
  }

  public static class FieldConstants {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);

    public static class RedSide {
      public static final Pose2d NET = new Pose2d(10.585, 3.198, Rotation2d.fromDegrees(-158.386));
      public static final Pose2d PROC = new Pose2d(11.534, 7.438, Rotation2d.fromDegrees(90));
      public static final Pose2d REEF_ONE = new Pose2d(11.700, 3.986, Rotation2d.fromDegrees(0));
      public static final Pose2d REEF_TWO = new Pose2d(12.314, 5.283, Rotation2d.fromDegrees(-60));
      public static final Pose2d REEF_THREE = new Pose2d(13.744, 5.283, Rotation2d.fromDegrees(-120));
      public static final Pose2d REEF_FOUR = new Pose2d(14.401, 4.031, Rotation2d.fromDegrees(180));
      public static final Pose2d REEF_FIVE = new Pose2d(13.744, 2.881, Rotation2d.fromDegrees(120));
      public static final Pose2d REEF_SIX = new Pose2d(12.398, 2.881, Rotation2d.fromDegrees(60));
      public static final Pose2d CORAL_STATION_RIGHT = new Pose2d(16.37, 7, Rotation2d.fromDegrees(54));
      public static final Pose2d CORAL_STATION_LEFT = new Pose2d(16.37, 1.05, Rotation2d.fromDegrees(-54));
      public static final Pose2d CAGE_ONE = new Pose2d(9.48, 0.811, Rotation2d.fromDegrees(0));
      public static final Pose2d CAGE_TWO = new Pose2d(9.48, 1.902, Rotation2d.fromDegrees(0));
      public static final Pose2d CAGE_THREE = new Pose2d(9.48, 2.952, Rotation2d.fromDegrees(0));
    }

    public static class BlueSide {
      public static final Pose2d NET = new Pose2d(6.8, 5.2, Rotation2d.fromDegrees(22));
      public static final Pose2d PROC = new Pose2d(6, 0.59, Rotation2d.fromDegrees(-90));
      public static final Pose2d REEF_ONE = new Pose2d(5.85, 3.9, Rotation2d.fromDegrees(180));
      public static final Pose2d REEF_TWO = new Pose2d(5.17, 2.87, Rotation2d.fromDegrees(120));
      public static final Pose2d REEF_THREE = new Pose2d(3.84, 2.873, Rotation2d.fromDegrees(60));
      public static final Pose2d REEF_FOUR = new Pose2d(3.14, 3.99, Rotation2d.fromDegrees(0));
      public static final Pose2d REEF_FIVE = new Pose2d(3.81, 5.18, Rotation2d.fromDegrees(-60));
      public static final Pose2d REEF_SIX = new Pose2d(5.15, 5.2, Rotation2d.fromDegrees(-120));
      public static final Pose2d CORAL_STATION_RIGHT = new Pose2d(1.19, 1.02, Rotation2d.fromDegrees(-125));
      public static final Pose2d CORAL_STATION_LEFT = new Pose2d(1.19, 7, Rotation2d.fromDegrees(125));
      public static final Pose2d CAGE_ONE = new Pose2d(7.98, 7.24, Rotation2d.fromDegrees(180));
      public static final Pose2d CAGE_TWO = new Pose2d(7.98, 6.22, Rotation2d.fromDegrees(180));
      public static final Pose2d CAGE_THREE = new Pose2d(7.98, 5.07, Rotation2d.fromDegrees(180));
    }
  }
}