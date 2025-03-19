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
public class ManualAlgaeL1 extends Command {

  private final AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private int step;
  private Timer waitTimer;

  /** Creates a new ManualCoralL3AlgaeL1. */
  public ManualAlgaeL1(AlgaeManipulatorSubsystem amSubsystem, ElevatorSubsystem eSubsystem) {
    this.algaeManipulatorSubsystem = amSubsystem;
    this.elevatorSubsystem = eSubsystem;

    addRequirements(amSubsystem, eSubsystem);
    this.waitTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.elevatorSubsystem.setElevatorTargetHeight(Level.ALGAE_L1);
    this.step = 0;
    this.waitTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!this.elevatorSubsystem.atTarget())
      return;

    this.algaeManipulatorSubsystem.intakeAlgae(0.5);
    switch (this.step) {
      case 0:
        this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(0));
        this.waitTimer.reset();
        this.waitTimer.start();
        this.step += 1;
        break;
      case 1:
        if (this.algaeManipulatorSubsystem.atTarget() && this.waitTimer.get() > 0.75)
          this.step += 1;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(85));
    this.algaeManipulatorSubsystem.holdAlgae();
    this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR);
    this.waitTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return step > 1;
  }
}
