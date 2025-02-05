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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private double currentTargetWristPosition = 0.5;
  private boolean isPIDControlling = true;
  private double operatorControlSpeed = 0;

  private double minRotationCount = 20;
  private double maxRotationCount = 65;
  private double minAbsoluteRotationCount = 0;
  private double maxAbsoluteRotationCount = 0.5;
  private double minWristAngle = 0;
  private double maxWristAngle = 90;

  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulatorSubsystem() {
    this.upperWheelMotor = new TalonFX(AlgaeManipulatorSubsystemConstants.UPPER_WHEEL_MOTOR_ID);
    this.lowerWheelMotor = new TalonFX(AlgaeManipulatorSubsystemConstants.LOWER_WHEEL_MOTOR_ID);
    

    SparkMaxConfig coralCurrentConfig = new SparkMaxConfig();
    coralCurrentConfig.smartCurrentLimit(20);
    this.coralMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.CORAL_MOTOR_ID, MotorType.kBrushless);
    this.coralMotor.configure(coralCurrentConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig wristCurrentLimit = new SparkMaxConfig();
    wristCurrentLimit.smartCurrentLimit(50);
    this.wristMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    this.wristMotor.configure(wristCurrentLimit, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.wristEncoder = this.wristMotor.getEncoder();
    this.absoluteWristEncoder = this.wristMotor.getAbsoluteEncoder();
    this.wristEncoder.setPosition(MathUtils.map(this.absoluteWristEncoder.getPosition(), minAbsoluteRotationCount, maxAbsoluteRotationCount, minRotationCount, maxRotationCount));
  }

  @Override
  public void periodic() {

    double currentWristPosition = getAbsoluteWristPosition();

    double error = this.currentTargetWristPosition - currentWristPosition;

    double kP = 0.5;

    double angleMotorOutput = kP * error;

    checkForOperatorOverride(angleMotorOutput);
    SmartDashboard.putNumber("Absolute position", getAbsoluteWristPosition());
  }

  public void intakeAlgae(double speed) {
    speed /= 2;
    this.moveUpperWheelMotorRaw(-speed);
    this.moveLowerWheelMotorRaw(speed);
  }

  public void outtakeAlgae(double speed) {
    speed /= 2;
    this.moveUpperWheelMotorRaw(speed);
    this.moveLowerWheelMotorRaw(-speed);
  }

  public void holdAlgae() {
    this.moveLowerWheelMotorRaw(0);
    this.moveUpperWheelMotorRaw(0);
  }

  private void moveUpperWheelMotorRaw(double speed) {
    upperWheelMotor.set(speed);
  }

  private void moveLowerWheelMotorRaw(double speed) {
    lowerWheelMotor.set(speed);
  }

  public void moveWristMotorRaw(double speed) {
    speed = Math.max(-0.5, Math.min(1, speed));
    if (this.getAbsoluteWristPosition() >= maxAbsoluteRotationCount && speed > 0)
      speed = 0;
    else if (this.getAbsoluteWristPosition() <= minAbsoluteRotationCount && speed < 0)
      speed = 0;
    speed = Math.max(-0.25, Math.min(1, speed));
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
    SmartDashboard.putNumber("Tilt Speed", speed);
  }

  public void checkForOperatorOverride(double pidControl) {
    double motorOutput = pidControl;

    if (!isPIDControlling) {
      motorOutput = operatorControlSpeed;
      currentTargetWristPosition = Math.max(minAbsoluteRotationCount, Math.min(getAbsoluteWristPosition(), maxAbsoluteRotationCount));
    }

    moveWristMotorRaw(motorOutput);
  }

  public double getAbsoluteWristPosition() {
    return absoluteWristEncoder.getPosition();
  }

  public Angle getWristAngleFromAbsolute(double rotations) {
    double degrees = MathUtils.map(rotations, minAbsoluteRotationCount, maxAbsoluteRotationCount, minWristAngle,
        maxWristAngle);
    return Angle.ofBaseUnits(degrees, Units.Degrees);
  }

  public double getAbsoluteFromWristAngle(Angle angle) {
    return MathUtils.map(angle.in(Units.Degrees), minWristAngle, maxWristAngle, minAbsoluteRotationCount, maxAbsoluteRotationCount);
  }

  public void setManipulatorAngle(Angle angle) {
    currentTargetWristPosition = getAbsoluteFromWristAngle(angle);
  }
}