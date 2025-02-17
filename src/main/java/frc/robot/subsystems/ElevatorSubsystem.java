// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants.Level;

public class ElevatorSubsystem extends SubsystemBase {

  private final AlgaeManipulatorSubsystem algaeManipulatorSubsystem;

  private SparkMax elevatorMotor;
  private SparkMax auxillaryElevatorMotor;
  private RelativeEncoder elevatorEncoder;

  private Level currentTargetLevel = Level.FLOOR;

  public boolean debugging = false;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(AlgaeManipulatorSubsystem amSubsystem) {
    this.algaeManipulatorSubsystem = amSubsystem;
    initializeMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!debugging) {
      double currentTargetPosition = this.getPositionFromLevel(currentTargetLevel);
      double currentPosition = elevatorEncoder.getPosition();
      double error = currentTargetPosition - currentPosition;
      double kP = 0.1; // Proportional gain, this value would need to be tuned
      double motorOutput = kP * error;

      double maxSpeed = 1;
      motorOutput = Math.max(-maxSpeed, Math.min(maxSpeed, motorOutput)); // Clamp the output between -1 and 1
      SmartDashboard.putNumber("Elevator/Elevator Controller Output", motorOutput);
      SmartDashboard.putString("Elevator/Current Target Level", this.currentTargetLevel.toString());
      SmartDashboard.putNumber("Elevator/Target Position", this.getPositionFromLevel(currentTargetLevel));
      SmartDashboard.putNumber("Elevator/Current Position", elevatorEncoder.getPosition());
      SmartDashboard.putBoolean("Elevator/At Target", this.atTarget());
      moveMotorRaw(motorOutput);
    }
  }

  public boolean atTarget() {
    return Math.abs(this.getPositionFromLevel(currentTargetLevel) - elevatorEncoder.getPosition()) < 2;
  }

  public Command moveElevatorToLevel(Level level) {
    return new FunctionalCommand(
        // Command init
        () -> this.setElevatorTargetHeight(level),
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

  private double heightToRawPosition(Distance height) {
    double inchesPerRotation = ElevatorSubsystemConstants.DISTANCE_PER_ROTATION.in(Units.Inches);
    double targetRotations = height.in(Units.Inches) / inchesPerRotation;
    return targetRotations * ElevatorSubsystemConstants.GEARING_MULTIPLIER;
  }

  public void moveDebugMotorRaw(double speed) {
    if (debugging)
      elevatorMotor.set(speed + 0.05);
  }

  private void moveMotorRaw(double speed) {
    if(!this.algaeManipulatorSubsystem.atTarget())
      speed = 0;
    if(this.elevatorEncoder.getPosition() < ElevatorSubsystemConstants.MAX_RAW_HEIGHT / 3)
      speed = Math.max(-0.2, speed);
    elevatorMotor.set(speed);
  }

  private double getPositionFromLevel(Level level) {
    Distance outputHeight;
    switch (level) {
      case FLOOR:
        outputHeight = ElevatorSubsystemConstants.FLOOR_LEVEL;
        break;
      case TROUGH:
        outputHeight = ElevatorSubsystemConstants.TROUGH_LEVEL;
        break;
      case CORAL_L2:
        outputHeight = ElevatorSubsystemConstants.CORAL_L2_LEVEL;
        break;
      case CORAL_L3:
        outputHeight = ElevatorSubsystemConstants.CORAL_L3_LEVEL;
        break;
      case CORAL_L4:
        outputHeight = ElevatorSubsystemConstants.CORAL_L4_LEVEL;
        break;
      case NET:
        outputHeight = ElevatorSubsystemConstants.NET_LEVEL;
        break;
      case ALGAE_L1:
        outputHeight = ElevatorSubsystemConstants.ALGAE_L1_LEVEL;
        break;
      case ALGAE_L2:
        outputHeight = ElevatorSubsystemConstants.ALGAE_L2_LEVEL;
        break;
      default:
        throw new UnsupportedOperationException("Unimplemented height " + level.name());
    }
    return heightToRawPosition(outputHeight);
  }

  /**
   * Sets the target height of the elevator.
   * Uses one of the pre-set heights (floor, trough, L2, L3, L4, and net) to set
   * the height.
   * 
   * @param height Value to set the height to
   */
  public boolean setElevatorTargetHeight(Level height) {
    this.currentTargetLevel = height;
    return true;
  }

  public boolean increaseLevel() {
    switch (currentTargetLevel) {
      case FLOOR:
        this.currentTargetLevel = Level.TROUGH;
        break;
      case TROUGH:
        this.currentTargetLevel = Level.CORAL_L2;
        break;
      case CORAL_L2:
        this.currentTargetLevel = Level.CORAL_L3;
        break;
      case CORAL_L3:
        this.currentTargetLevel = Level.ALGAE_L2;
        break;
      case ALGAE_L2:
        this.currentTargetLevel = Level.NET;
        break;
      default:
        return false; // Already at the highest level
    }
    return true;
  }

  public boolean decreaseLevel() {
    switch (currentTargetLevel) {
      case NET:
        this.currentTargetLevel = Level.CORAL_L4;
        break;
      case CORAL_L4:
        this.currentTargetLevel = Level.CORAL_L3;
        break;
      case CORAL_L3:
        this.currentTargetLevel = Level.CORAL_L2;
        break;
      case CORAL_L2:
        this.currentTargetLevel = Level.TROUGH;
        break;
      case TROUGH:
        this.currentTargetLevel = Level.FLOOR;
        break;
      default:
        return false; // Already at the lowest level
    }
    return true;
  }

  private void initializeMotors() {
    elevatorMotor = new SparkMax(ElevatorSubsystemConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);
    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorEncoder.setPosition(0);

    auxillaryElevatorMotor = new SparkMax(ElevatorSubsystemConstants.AUXILLARY_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig auxElevatorMotorConfig = new SparkMaxConfig();
    auxElevatorMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .follow(elevatorMotor, true);
    auxillaryElevatorMotor.configure(auxElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
