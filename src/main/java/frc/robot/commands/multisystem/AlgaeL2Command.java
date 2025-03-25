// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multisystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorSubsystemConstants.Level;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeL2Command extends Command {

  private final AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private int step;
  private Timer waitTimer;

  /** Creates a new ManualCoralL3AlgaeL1. */
  public AlgaeL2Command(AlgaeManipulatorSubsystem amSubsystem, ElevatorSubsystem eSubsystem) {
    this.algaeManipulatorSubsystem = amSubsystem;
    this.elevatorSubsystem = eSubsystem;
    this.waitTimer = new Timer();

    addRequirements(amSubsystem, eSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.elevatorSubsystem.setElevatorTargetHeight(Level.CORAL_L3);
    this.step = 0;
    this.waitTimer.reset();
    this.waitTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!this.elevatorSubsystem.atTarget())
      return;

    this.algaeManipulatorSubsystem.intakeAlgae(0.5);
    switch(this.step) {
      case 0:
        this.elevatorSubsystem.setElevatorTargetHeight(Level.ALGAE_L2);
        if(this.elevatorSubsystem.atTarget()) {
          this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(45));
          this.step += 1;
          this.waitTimer.restart();
        }
        break;
      case 1:
        if(this.algaeManipulatorSubsystem.atTarget() && this.waitTimer.get() > 0.25) {
          this.step += 1;
          this.waitTimer.reset();
          this.waitTimer.start();
        }
        break;
      case 2:
        if (this.waitTimer.get() > 0.75) {
          this.step += 1;
          this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR);
          this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(85));
          this.waitTimer.reset();
          this.waitTimer.start();
        }
        break;
      case 3:
        if (this.waitTimer.get() > 0.25)
          this.step += 1;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.algaeManipulatorSubsystem.holdAlgae();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  step > 3;
  }
}
