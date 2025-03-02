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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

  private final double minRotationCount = 20;
  private final double maxRotationCount = 65;
  private final double minAbsoluteRotationCount = 0;
  private final double maxAbsoluteRotationCount = 0.49;
  private final double minWristAngle = 0;
  private final double maxWristAngle = 90;

  private PIDController wristController;

  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulatorSubsystem() {
    this.upperWheelMotor = new TalonFX(AlgaeManipulatorSubsystemConstants.UPPER_WHEEL_MOTOR_ID);
    this.upperWheelMotor.setNeutralMode(NeutralModeValue.Brake);
    this.lowerWheelMotor = new TalonFX(AlgaeManipulatorSubsystemConstants.LOWER_WHEEL_MOTOR_ID);
    this.lowerWheelMotor.setNeutralMode(NeutralModeValue.Brake);

    SparkMaxConfig coralCurrentConfig = new SparkMaxConfig();
    coralCurrentConfig.smartCurrentLimit(20);
    this.coralMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.CORAL_MOTOR_ID, MotorType.kBrushless);
    this.coralMotor.configure(coralCurrentConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig wristCurrentLimit = new SparkMaxConfig();
    wristCurrentLimit.smartCurrentLimit(50);
    SoftLimitConfig wristLimits = new SoftLimitConfig();
    wristLimits.forwardSoftLimit(this.maxRotationCount);
    wristLimits.forwardSoftLimitEnabled(false);
    wristLimits.reverseSoftLimit(this.minRotationCount - 20);
    wristLimits.reverseSoftLimitEnabled(false);
    wristCurrentLimit.apply(wristLimits);
    this.wristMotor = new SparkMax(AlgaeManipulatorSubsystemConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    this.wristMotor.configure(wristCurrentLimit, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.wristEncoder = this.wristMotor.getEncoder();
    this.absoluteWristEncoder = this.wristMotor.getAbsoluteEncoder();
    this.wristEncoder.setPosition(MathUtils.map(this.absoluteWristEncoder.getPosition(), minAbsoluteRotationCount,
        maxAbsoluteRotationCount, minRotationCount, maxRotationCount));
    this.wristController = new PIDController(5, 0.000001, 0); // We don't actually use kI, but we do use it's error
                                                              // detection which is disabled when set to 0. So instead
                                                              // we set it to a very low number
    this.wristController.setTolerance(0.04);
    this.wristController.setIntegratorRange(-0.3, 0.03);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist/Target Position", currentTargetWristPosition);
    SmartDashboard.putNumber("Wrist/Current Position", getAbsoluteWristPosition());
    SmartDashboard.putBoolean("Wrist/At Target", this.atTarget());
    SmartDashboard.putNumber("Wrist/Accumulated Error", this.wristController.getAccumulatedError());
    SmartDashboard.putBoolean("Wrist/Stalled", isStalled());
    double angleMotorOutput = this.wristController.calculate(getAbsoluteWristPosition(),
        this.currentTargetWristPosition);
    checkForOperatorOverride(angleMotorOutput);
  }

  public void resetClawError() {
    this.wristController.reset();
    this.wristController.setTolerance(0.04);
    this.wristController.setIntegratorRange(-0.3, 0.03);
  }

  public boolean isStalled() {
    return Math.abs(this.wristController.getAccumulatedError()) > 0.25;
  }

  public void intakeAlgae(double speed) {
    this.moveUpperWheelMotorRaw(speed);
    this.moveLowerWheelMotorRaw(-speed);
  }

  public void outtakeAlgae(double speed) {
    this.moveUpperWheelMotorRaw(-speed);
    this.moveLowerWheelMotorRaw(speed);
  }

  public void holdAlgae() {
    this.moveUpperWheelMotorRaw(0);
    this.moveLowerWheelMotorRaw(0);
  }

  public boolean atTarget() {
    return this.wristController.atSetpoint() || this.isStalled();
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

  private void moveUpperWheelMotorRaw(double speed) {
    upperWheelMotor.set(speed);
  }

  private void moveLowerWheelMotorRaw(double speed) {
    lowerWheelMotor.set(speed);
  }

  public void moveWristMotorRaw(double speed) {
    if (speed > 0 && this.absoluteWristEncoder.getPosition() >= this.maxAbsoluteRotationCount)
      speed = 0;
    if (speed < 0 && this.absoluteWristEncoder.getPosition() <= this.minAbsoluteRotationCount)
      speed = 0;
    wristMotor.set(speed);
  }

  public void intakeCoral() {
    this.moveCoralMotorRaw(-1);
  }

  public void outtakeCoral() {
    this.moveCoralMotorRaw(1);
  }

  public void holdCoral() {
    this.moveCoralMotorRaw(0);
  }

  private void moveCoralMotorRaw(double speed) {
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
      currentTargetWristPosition = Math.max(minAbsoluteRotationCount,
          Math.min(getAbsoluteWristPosition(), maxAbsoluteRotationCount));
    }

    moveWristMotorRaw(motorOutput);
  }

  public double getAbsoluteWristPosition() {
    double pos = absoluteWristEncoder.getPosition();
    if (pos > 0.9)
      pos = 0 - (1 - pos);
    return pos;
  }

  public Angle getWristAngleFromAbsolute(double rotations) {
    double degrees = MathUtils.map(rotations, minAbsoluteRotationCount, maxAbsoluteRotationCount, minWristAngle,
        maxWristAngle);
    return Angle.ofBaseUnits(degrees, Units.Degrees);
  }

  public double getAbsoluteFromWristAngle(Angle angle) {
    return MathUtils.map(angle.in(Units.Degrees), minWristAngle, maxWristAngle, minAbsoluteRotationCount,
        maxAbsoluteRotationCount);
  }

  public double getWristMotorRotations(Angle angle) {
    double degrees = angle.in(Units.Degrees);
    return MathUtils.map(degrees, minWristAngle, maxWristAngle, minRotationCount, maxRotationCount);
  }

  public void setManipulatorAngle(Angle angle) {
    currentTargetWristPosition = getAbsoluteFromWristAngle(angle);
    this.resetClawError();
  }
}