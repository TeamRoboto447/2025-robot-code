// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multisystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorSubsystemConstants.Level;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualAlgaeProcessor extends Command {
  private final AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  /** Creates a new ManualAlgaeProcessor. */
  public ManualAlgaeProcessor(AlgaeManipulatorSubsystem amSubsystem, ElevatorSubsystem eSubsystem) {
    this.algaeManipulatorSubsystem = amSubsystem;
    this.elevatorSubsystem = eSubsystem;

    addRequirements(amSubsystem, eSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR);
    this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(60));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.algaeManipulatorSubsystem.atTarget() && this.elevatorSubsystem.atTarget()) {
      this.algaeManipulatorSubsystem.outtakeAlgae(0.5);
    } else {
      this.algaeManipulatorSubsystem.holdAlgae();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(90));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
