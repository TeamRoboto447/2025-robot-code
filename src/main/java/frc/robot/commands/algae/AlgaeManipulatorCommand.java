// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeManipulatorCommand extends Command {
  private final AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private final CommandXboxController operatorController;
  /** Creates a new AlgaeManipulatorCommand. */
  public AlgaeManipulatorCommand(AlgaeManipulatorSubsystem aManipulatorSubsystem, CommandXboxController oController) {
    this.algaeManipulatorSubsystem = aManipulatorSubsystem;
    this.operatorController = oController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.algaeManipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.operatorController.getLeftTriggerAxis() > 0.1) {
      this.algaeManipulatorSubsystem.moveUpperWheelMotorRaw(this.operatorController.getLeftTriggerAxis());
      this.algaeManipulatorSubsystem.moveLowerWheelMotorRaw(-this.operatorController.getLeftTriggerAxis());
    } else if (this.operatorController.getRightTriggerAxis() > 0.1) {
      this.algaeManipulatorSubsystem.moveUpperWheelMotorRaw(-this.operatorController.getRightTriggerAxis());
      this.algaeManipulatorSubsystem.moveLowerWheelMotorRaw(this.operatorController.getRightTriggerAxis());
    } else {
      this.algaeManipulatorSubsystem.moveUpperWheelMotorRaw(0);
      this.algaeManipulatorSubsystem.moveLowerWheelMotorRaw(0);
    }

    this.algaeManipulatorSubsystem.setOperatorRequestedSpeed(this.operatorController.getLeftY());
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
