// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;
  private Double currentSpeed = 0.0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    initializeMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    moveMotorRaw(currentSpeed);
  }

  private void moveMotorRaw(double speed) {
    elevatorMotor.set(speed);
  }

  public void stopEverything() {
    elevatorMotor.stopMotor();
  }

  public boolean setSpeed(double speed) {
    if (speed > 1 || speed < -1) {return false;}
    currentSpeed = speed;
    return true;
  }

  private void initializeMotors() {
    elevatorMotor = new SparkMax(ElevatorSubsystemConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder();
  }
}
