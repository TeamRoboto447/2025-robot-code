// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorSubsystemConstants.Level;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorControlCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final CommandXboxController operatorController;
  /** Creates a new ElevatorControlCommand. */
  public ElevatorControlCommand(ElevatorSubsystem eSubsystem, CommandXboxController oController) {
    this.elevatorSubsystem = eSubsystem;
    this.operatorController = oController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.operatorController.getRightTriggerAxis() > 0.5) {
      this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR);
    } else if (this.operatorController.a().getAsBoolean()) {
      this.elevatorSubsystem.setElevatorTargetHeight(Level.TROUGH);
    } else if (this.operatorController.b().getAsBoolean()) {
      this.elevatorSubsystem.setElevatorTargetHeight(Level.CORAL_L2);
    } else if (this.operatorController.x().getAsBoolean()) {
      this.elevatorSubsystem.setElevatorTargetHeight(Level.CORAL_L3);
    } else if (this.operatorController.y().getAsBoolean()) {
      this.elevatorSubsystem.setElevatorTargetHeight(Level.CORAL_L4);
    } else if (this.operatorController.rightBumper().getAsBoolean()) {
      this.elevatorSubsystem.setElevatorTargetHeight(Level.NET);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
