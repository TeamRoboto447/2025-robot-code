// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

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
    public static final double MAX_SPEED = Units.feetToMeters(4);
    public static final double ROBOT_MASS = Units.lbsToKilograms(96.6); // TODO: update mass with actual value
    public static final Matter CHASSIS = new Matter(new Translation3d(0.0, 0.0, Units.inchesToMeters(8)), ROBOT_MASS); //TODO: update with actual robot size
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
  }

  public static class ElevatorSubsystemConstants {
    public static final int ELEVATOR_MOTOR_ID = 22;
    public static final int ELEVATOR_UPPER_LIMIT_SWITCH_CHANNEL = 2;
    public static final int ELEVATOR_LOWER_LIMIT_SWITCH_CHANNEL = 3;

    public enum Level {
      FLOOR,
      TROUGH,
      CORAL_L2,
      CORAL_L3,
      CORAL_L4,
      ALGAE_L1,
      ALGAE_L2,
      NET
    }

    public static final double MIN_INCH_HEIGHT = 11.5;
    public static final double MAX_INCH_HEIGHT = 92;
    public static final int MIN_RAW_HEIGHT = 0;
    public static final int MAX_RAW_HEIGHT = 270;

    public static final Distance FLOOR_LEVEL = Inches.of(0 + MIN_INCH_HEIGHT);
    public static final Distance TROUGH_LEVEL = Inches.of(0 + MIN_INCH_HEIGHT);
    public static final Distance CORAL_L2_LEVEL = Inches.of(13);
    public static final Distance CORAL_L3_LEVEL = Inches.of(29);
    public static final Distance ALGAE_L1_LEVEL = CORAL_L3_LEVEL;
    public static final Distance ALGAE_L2_LEVEL = Inches.of(45);
    public static final Distance CORAL_L4_LEVEL = Inches.of(66);
    public static final Distance NET_LEVEL = Inches.of(80);
  }

  public static class AlgaeManipulatorSubsystemConstants {
    public static final int UPPER_WHEEL_MOTOR_ID = 24;
    public static final int LOWER_WHEEL_MOTOR_ID = 25;
    public static final int WRIST_MOTOR_ID = 26;
    public static final int CORAL_MOTOR_ID = 27;
  }
}
