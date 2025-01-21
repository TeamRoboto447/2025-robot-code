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
  private RelativeEncoder upperWheelEncoder;
  private RelativeEncoder lowerWheelEncoder;
  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulatorSubsystem() {
    this.upperWheelMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.UPPER_WHEEL_MOTOR_ID, MotorType.kBrushless);
    this.lowerWheelMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.UPPER_WHEEL_MOTOR_ID, MotorType.kBrushless);
    this.upperWheelEncoder = upperWheelMotor.getEncoder();
    this.lowerWheelEncoder = lowerWheelMotor.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}




