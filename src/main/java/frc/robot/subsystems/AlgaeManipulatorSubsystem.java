// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/**The Subsystem will make use of three motors to intake, manipulate, and output both algae and coral 
 * the first motor will rotate clockwise and counterclockwise motion to pull in and put out algae:
 *  I intend to make use of positive and negative output values in order to identify motor direction 
 * The second motor will rotate either dirrection to pull in and put out algae and coral 
 * The third motor will swivel to adjust the verticle angle of the entire assembly 
 */
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManipulatorSubsystemConstants;

public class AlgaeManipulatorSubsystem extends SubsystemBase {
  private SparkMax upperWheelMotor;
  private SparkMax lowerWheelMotor;
  private SparkMax angleMotor;
  private RelativeEncoder upperWheelEncoder;
  private RelativeEncoder lowerWheelEncoder;
  private RelativeEncoder angleEncoder;

  private double currentTargetAngle = 0.0;
  private boolean isPIDControlling = true;
  private double operatorControlSpeed = 0;

  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulatorSubsystem() {
    this.upperWheelMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.UPPER_WHEEL_MOTOR_ID, MotorType.kBrushless);
    this.lowerWheelMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.UPPER_WHEEL_MOTOR_ID, MotorType.kBrushless);
    this.angleMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
    this.upperWheelEncoder = upperWheelMotor.getEncoder();
    this.lowerWheelEncoder = lowerWheelMotor.getEncoder();
    this.angleEncoder = angleMotor.getEncoder();

  }

  @Override
  public void periodic() {

    double currentAngle = this.angleEncoder.getPosition();

    double error = this.currentTargetAngle - currentAngle;

    double kP = 0.1;

    double angleMotorOutput = kP * error;

    angleMotorOutput = Math.max(-1, Math.min(1, angleMotorOutput));
    
    // if (this.AlgaeManipulatorUpperLimitSwitch.get() && angleMotorOutput > 0) {
    //   angleMotorOutput = 0;
    // }

    // if (this.AlgaeManipulatorLowerLimitSwitch.get() && angleMotorOutput < 0) {
    //   angleMotorOutput = 0;
    // }

    checkForOperatorOverride(angleMotorOutput);
  }

  public void moveUpperWheelMotorRaw(double upperWheelMotorSpeed) {
    upperWheelMotor.set(upperWheelMotorSpeed);
  }

  public void moveLowerWheelMotorRaw(double lowerWheelMotorSpeed) {
    upperWheelMotor.set(lowerWheelMotorSpeed);
  }

  public void moveAngleMotorRaw(double angleMotorSpeed) {
    upperWheelMotor.set(angleMotorSpeed);
  }

  public void setIsPIDControlled(boolean enabled) {
    isPIDControlling = enabled;
  }

  public void setOperatorRequestedSpeed(double speed) {
    operatorControlSpeed = speed;
  }

  public void checkForOperatorOverride(double pidControl) {
    double motorOutput = pidControl;

    if (!isPIDControlling) {
      motorOutput = operatorControlSpeed;
      currentTargetAngle = angleEncoder.getPosition();
    }

    moveAngleMotorRaw(motorOutput);
  }
}