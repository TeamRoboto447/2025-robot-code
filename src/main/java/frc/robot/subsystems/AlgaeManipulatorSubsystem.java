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

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManipulatorSubsystemConstants;
import frc.robot.utils.MathUtils;

public class AlgaeManipulatorSubsystem extends SubsystemBase {
  private final TalonFX upperWheelMotor;
  private final TalonFX lowerWheelMotor;
  private final SparkMax wristMotor;
  private final SparkMax coralMotor;

  private final RelativeEncoder wristEncoder;
  private final AbsoluteEncoder absoluteWristEncoder;

  private final double absoluteEncoderOffset = 0.993;

  private double currentTargetWristPosition = 0.0;
  private boolean isPIDControlling = true;
  private double operatorControlSpeed = 0;

  private double minRotationCount = 0;
  private double maxRotationCount = 10;
  private double minAbsoluteRotationCount = 0;
  private double maxAbsoluteRotationCount = 0.5;
  private double minWristAngle = 0;
  private double maxWristAngle = 90;

  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulatorSubsystem() {
    this.upperWheelMotor = new TalonFX(AlgaeManipulatorSubsystemConstants.UPPER_WHEEL_MOTOR_ID);
    this.lowerWheelMotor = new TalonFX(AlgaeManipulatorSubsystemConstants.LOWER_WHEEL_MOTOR_ID);
    this.coralMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.CORAL_MOTOR_ID, MotorType.kBrushless);
    
    this.wristMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    SoftLimitConfig wristLimits = new SoftLimitConfig();
    wristLimits.forwardSoftLimit(maxRotationCount);
    wristLimits.forwardSoftLimitEnabled(true);
    wristLimits.reverseSoftLimit(minRotationCount);
    wristLimits.reverseSoftLimitEnabled(true);
    wristConfig.apply(wristLimits);
    wristConfig.idleMode(IdleMode.kBrake);
    this.wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.wristEncoder = this.wristMotor.getEncoder();
    this.absoluteWristEncoder = this.wristMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {

    double error = this.currentTargetWristPosition - getAbsoluteWristPosition();

    double kP = 0.1;

    double angleMotorOutput = kP * error;

    checkForOperatorOverride(angleMotorOutput);
  }

  public boolean atTarget() {
    return Math.abs(this.currentTargetWristPosition - getAbsoluteWristPosition()) < 0.05;
  }

  public Command tiltToAngle(Angle angle) {
    return new FunctionalCommand(
        // Command init
        () -> this.setManipulatorAngle(angle),
        // Command execute/periodic
        () -> {
        },
        // Command end
        interrupted -> {
        },
        // Command isFinished
        () -> this.atTarget(),
        // Command requirements
        this);
  }

  public void moveUpperWheelMotorRaw(double speed) {
    upperWheelMotor.set(speed);
  }

  public void moveLowerWheelMotorRaw(double speed) {
    lowerWheelMotor.set(speed);
  }

  public void moveWristMotorRaw(double speed) {
    speed = Math.max(-0.1, Math.min(0.2, speed));
    wristMotor.set(speed);
  }

  public void moveCoralMotorRaw(double speed) {
    coralMotor.set(speed);
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
      currentTargetWristPosition = getAbsoluteWristPosition();
    }

    moveWristMotorRaw(motorOutput);
  }

  public double getWristMotorPosition() {
    return wristEncoder.getPosition();
  }

  public double getAbsoluteWristPosition() {
    return absoluteWristEncoder.getPosition() - absoluteEncoderOffset;
  }

  public Angle getWristAngleFromAbsolute(double rotations) {
    double degrees = MathUtils.map(rotations, minAbsoluteRotationCount, maxAbsoluteRotationCount, minWristAngle, maxWristAngle);
    return Angle.ofBaseUnits(degrees, Units.Degrees);
  }

  public Angle getWristAngleFromRelative(double rotations) {
    double degrees = MathUtils.map(rotations, minRotationCount, maxRotationCount, minWristAngle, maxWristAngle);
    return Angle.ofBaseUnits(degrees, Units.Degrees);
  }

  public double getWristMotorRotations(Angle angle) {
    double degrees = angle.in(Units.Degrees);
    return MathUtils.map(degrees, minWristAngle, maxWristAngle, minRotationCount, maxRotationCount);
  }

  public double getWristAbsolutePosition(Angle angle) {
    double degrees = angle.in(Units.Degrees);
    return MathUtils.map(degrees, minWristAngle, maxWristAngle, minAbsoluteRotationCount, maxAbsoluteRotationCount);
  }

  public void setManipulatorAngle(Angle angle) {
    currentTargetWristPosition = getWristAbsolutePosition(angle);
  }
}