// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystemConstants;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax climberMotor;

  @SuppressWarnings("unused")
  private RelativeEncoder ClimberEncoder; // TODO: Implement this
  private Double currentSpeed = 0.0;

  private double minRotationCount = 0;
  private double maxRotationCount = 190;
  private DigitalInput lowerLimitSwitch;

  /** Creates a new climberSubsystem. */
  public ClimberSubsystem() {
    initializeMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber/Position", this.climberMotor.getEncoder().getPosition());
    moveMotorRaw(currentSpeed);
  }

  private void moveMotorRaw(double speed) {
    SmartDashboard.putNumber("ClimberPosition", this.ClimberEncoder.getPosition());
    if (lowerLimitSwitch.get()) {
      this.ClimberEncoder.setPosition(0);
      if (speed < 0)
        speed = 0;
    }
    climberMotor.set(speed);
  }

  public void stopEverything() {
    climberMotor.stopMotor();
  }

  public boolean setSpeed(double speed) {
    if (speed > 1 || speed < -1) {
      return false;
    }
    currentSpeed = speed;
    return true;
  }

  private void initializeMotors() {
    climberMotor = new SparkMax(ClimberSubsystemConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    SoftLimitConfig motorLimits = new SoftLimitConfig();
    motorLimits.forwardSoftLimit(maxRotationCount);
    motorLimits.forwardSoftLimitEnabled(true);
    motorLimits.reverseSoftLimit(minRotationCount);
    motorLimits.reverseSoftLimitEnabled(false);
    // motorConfig.apply(motorLimits);
    climberMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    ClimberEncoder = climberMotor.getEncoder();

    lowerLimitSwitch = new DigitalInput(ClimberSubsystemConstants.CLIMBER_LOWER_LIMIT_ID);
  }
}
