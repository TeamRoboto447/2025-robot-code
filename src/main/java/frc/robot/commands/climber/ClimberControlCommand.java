// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberControlCommand extends Command {

  private final ClimberSubsystem climberSubsystem;
  private final CommandXboxController driverController;
  /** Creates a new climberControl. */
  public ClimberControlCommand(ClimberSubsystem eSubsystem, CommandXboxController dController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberSubsystem = eSubsystem;
    this.driverController = dController;

    addRequirements(this.climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 1;

    if (this.driverController.rightTrigger().getAsBoolean()) {
      this.climberSubsystem.setSpeed(-speed);
    } else if (this.driverController.leftTrigger().getAsBoolean()) {
      this.climberSubsystem.setSpeed(speed);
    } else {
      this.climberSubsystem.setSpeed(0.0);
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
