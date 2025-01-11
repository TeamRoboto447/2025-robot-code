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
    initializeMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    double currentPosition = exampleEncoder.getPosition();
    double error = currentTargetPosition - currentPosition;
    double kP = 0.1; // Proportional gain, this value would need to be tuned
    double motorOutput = kP * error;
    motorOutput = Math.max(-1, Math.min(1, motorOutput)); // Clamp the output between -1 and 1

    if (exampleUpperLimitSwitch.get() && motorOutput > 0) {
      motorOutput = 0; // Stop motor if upper limit switch is hit and output is positive
    }
    if (exampleLowerLimitSwitch.get() && motorOutput < 0) {
      motorOutput = 0; // Stop motor if lower limit switch is hit and output is negative
    }

    moveMotorRaw(motorOutput);
  }

  public Boolean setTargetPosition(double position) {
    if(position < minimumPosition || position > maximumPosition) {
      return false;
    }
    this.currentTargetPosition = position;
    return true;
  }

  public double getCurrentPosition() {
    return exampleEncoder.getPosition();
  }

  public double getTargetPosition() {
    return currentTargetPosition;
  }

  private void moveMotorRaw(double speed) {
    exampleMotor.set(speed);
  }

  public void stopEverything() {
    exampleMotor.stopMotor();
  }

  private void initializeMotors() {
    exampleMotor = new SparkMax(ExampleSubsystemConstants.kExampleMotorID, MotorType.kBrushless);
    exampleEncoder = exampleMotor.getEncoder();
    exampleLowerLimitSwitch = new DigitalInput(ExampleSubsystemConstants.kExampleLowerLimitSwitchChannel);
    exampleUpperLimitSwitch = new DigitalInput(ExampleSubsystemConstants.kExampleUpperLimitSwitchChannel);
  }
}
