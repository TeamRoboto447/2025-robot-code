// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants.Level;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;
  private DigitalInput elevatorUpperLimitSwitch;
  private DigitalInput elevatorLowerLimitSwitch;

  private Level currentLevel = Level.FLOOR;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    initializeMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentTargetPosition = this.getPositionFromLevel(currentLevel);
    double currentPosition = elevatorEncoder.getPosition();
    double error = currentTargetPosition - currentPosition;
    double kP = 0.1; // Proportional gain, this value would need to be tuned
    double motorOutput = kP * error;
    motorOutput = Math.max(-1, Math.min(1, motorOutput)); // Clamp the output between -1 and 1

    // if (elevatorUpperLimitSwitch.get() && motorOutput > 0) {
    //   motorOutput = 0; // Stop motor if upper limit switch is hit and output is positive
    // }
    // if (elevatorLowerLimitSwitch.get() && motorOutput < 0) {
    //   motorOutput = 0; // Stop motor if lower limit switch is hit and output is negative
    // }

    moveMotorRaw(motorOutput);
  }

  private void moveMotorRaw(double speed) {
    elevatorMotor.set(speed);
  }

  private double getPositionFromLevel(Level level) {
    switch (level) {
      case FLOOR:
        return ElevatorSubsystemConstants.FLOOR_LEVEL;
      case TROUGH:
        return ElevatorSubsystemConstants.TROUGH_LEVEL;
      case L2:
        return ElevatorSubsystemConstants.L2_LEVEL;
      case L3:
        return ElevatorSubsystemConstants.L3_LEVEL;
      case L4:
        return ElevatorSubsystemConstants.L4_LEVEL;
      case NET:
        return ElevatorSubsystemConstants.NET_LEVEL;
      default:
        return 0;
    }
  }

  /**
   * Sets the target height of the elevator.
   * Uses one of the pre-set heights (floor, trough, L2, L3, L4, and net) to set
   * the height.
   * 
   * @param height Value to set the height to
   */
  public boolean setElevatorTargetHeight(Level height) {
    this.currentLevel = height;
    return true;
  }

  public boolean increaseLevel() {
    switch (currentLevel) {
      case FLOOR:
        this.currentLevel = Level.TROUGH;
        break;
      case TROUGH:
        this.currentLevel = Level.L2;
        break;
      case L2:
        this.currentLevel = Level.L3;
        break;
      case L3:
        this.currentLevel = Level.L4;
        break;
      case L4:
        this.currentLevel = Level.NET;
        break;
      default:
        return false; // Already at the highest level
    }
    return true;
  }

  public boolean decreaseLevel() {
    switch (currentLevel) {
      case NET:
        this.currentLevel = Level.L4;
        break;
      case L4:
        this.currentLevel = Level.L3;
        break;
      case L3:
        this.currentLevel = Level.L2;
        break;
      case L2:
        this.currentLevel = Level.TROUGH;
        break;
      case TROUGH:
        this.currentLevel = Level.FLOOR;
        break;
      default:
        return false; // Already at the lowest level
    }
    return true;
  }

  private void initializeMotors() {
    elevatorMotor = new SparkMax(ElevatorSubsystemConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorUpperLimitSwitch = new DigitalInput(ElevatorSubsystemConstants.ELEVATOR_UPPER_LIMIT_SWITCH_CHANNEL);
    elevatorLowerLimitSwitch = new DigitalInput(ElevatorSubsystemConstants.ELEVATOR_LOWER_LIMIT_SWITCH_CHANNEL);
  }
}
