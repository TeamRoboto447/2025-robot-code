// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExampleSubsystemConstants;

public class ExampleSubsystem extends SubsystemBase {
  private SparkMax exampleMotor;
  private RelativeEncoder exampleEncoder;
  private DigitalInput exampleLowerLimitSwitch;
  private DigitalInput exampleUpperLimitSwitch;

  private double currentTargetPosition = 0;
  private double minimumPosition = 0;
  private double maximumPosition = 1000;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    this.exampleMotor = new SparkMax(ExampleSubsystemConstants.EXAMPLE_MOTOR_ID, MotorType.kBrushless); // Initialize a SparkMax motor controller with a brushless motor
    this.exampleEncoder = exampleMotor.getEncoder(); // Get the encoder object from the motor controller
    this.exampleLowerLimitSwitch = new DigitalInput(ExampleSubsystemConstants.EXAMPLE_LOWER_LIMIT_SWITCH_CHANNEL); // Initialize the lower limit switch
    this.exampleUpperLimitSwitch = new DigitalInput(ExampleSubsystemConstants.EXAMPLE_UPPER_LIMIT_SWITCH_CHANNEL); // Initialize the upper limit switch
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    double currentPosition = this.exampleEncoder.getPosition(); // Get the current position of the motor

    double error = this.currentTargetPosition - currentPosition; // Calculate the error between the current position and the target position

    double kP = 0.1; // Proportional gain constant, tune this value to get the desired performance

    double motorOutput = kP * error; // Multiply the error by the gain constant to get the motor output

    motorOutput = Math.max(-1, Math.min(1, motorOutput)); // Clamp the output between -1 and 1

    if (this.exampleUpperLimitSwitch.get() && motorOutput > 0) { // Check if the upper limit switch is hit and the output is positive
      motorOutput = 0; // Stop motor if upper limit switch is hit and output is positive
    }

    if (this.exampleLowerLimitSwitch.get() && motorOutput < 0) { // Check if the lower limit switch is hit and the output is negative
      motorOutput = 0; // Stop motor if lower limit switch is hit and output is negative
    }

    moveMotorRaw(motorOutput); // Move the motor with the calculated output
  }

  public Boolean setTargetPosition(double position) {
    if(position < minimumPosition || position > maximumPosition) { // Check if the target position is within the allowed range
      return false; // Return false if the target position is out of range, indicating that we failed to set the position
    }

    this.currentTargetPosition = position; // Set the target position
    return true; // Return true if the target position was set successfully
  }

  public double getCurrentPosition() {
    return exampleEncoder.getPosition(); // Return the current position of the motor
  }

  public double getTargetPosition() {
    return currentTargetPosition; // Return the target position of the motor
  }

  private void moveMotorRaw(double speed) {
    exampleMotor.set(speed); // Set the motor speed
  }

  public void stopEverything() {
    exampleMotor.stopMotor(); // Stop the motor, this should be called any time a command that uses the motor ends
  }

}
