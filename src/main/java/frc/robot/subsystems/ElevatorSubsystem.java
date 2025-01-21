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

  private double currentTargetPosition = 0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    initializeMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPosition = elevatorEncoder.getPosition();
    double error = currentTargetPosition - currentPosition;
    double kP = 0.1; // Proportional gain, this value would need to be tuned
    double motorOutput = kP * error;
    motorOutput = Math.max(-1, Math.min(1, motorOutput)); // Clamp the output between -1 and 1

    if (elevatorUpperLimitSwitch.get() && motorOutput > 0) {
      motorOutput = 0; // Stop motor if upper limit switch is hit and output is positive
    }
    if (elevatorLowerLimitSwitch.get() && motorOutput < 0) {
      motorOutput = 0; // Stop motor if lower limit switch is hit and output is negative
    }

    moveMotorRaw(motorOutput);
  }

  private void moveMotorRaw(double speed) {
    elevatorMotor.set(speed);
  }

  /**
   * Sets the target height of the elevator.
   * Uses one of the pre-set heights (floor, trough, L2, L3, L4, and net) to set
   * the height.
   * 
   * @param height Value to set the height to
   */
  public boolean setElevatorTargetHeight(Level height) {
    switch (height) {
      case FLOOR:
        currentTargetPosition = ElevatorSubsystemConstants.FLOOR_LEVEL;
        break;
      case TROUGH:
        currentTargetPosition = ElevatorSubsystemConstants.TROUGH_LEVEL;
        break;
      case L2:
        currentTargetPosition = ElevatorSubsystemConstants.L2_LEVEL;
        break;
      case L3:
        currentTargetPosition = ElevatorSubsystemConstants.L3_LEVEL;
        break;
      case L4:
        currentTargetPosition = ElevatorSubsystemConstants.L4_LEVEL;
        break;
      case NET:
        currentTargetPosition = ElevatorSubsystemConstants.NET_LEVEL;
        break;
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
