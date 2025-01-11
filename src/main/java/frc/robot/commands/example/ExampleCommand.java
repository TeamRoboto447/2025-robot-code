// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.example;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ExampleSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ExampleCommand extends Command {
  /** Creates a new ExampleCommand. */
  private final ExampleSubsystem exampleSubsystem;
  private final CommandXboxController operatorController;

  public ExampleCommand(ExampleSubsystem eSubsystem, CommandXboxController oController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.exampleSubsystem = eSubsystem;
    this.operatorController = oController;
    addRequirements(this.exampleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorController.b().getAsBoolean()) {
      exampleSubsystem.setTargetPosition(1000);
    } else if (operatorController.a().getAsBoolean()) {
      exampleSubsystem.setTargetPosition(0);
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
