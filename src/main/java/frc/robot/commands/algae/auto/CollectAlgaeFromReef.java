// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae.auto;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectAlgaeFromReef extends Command {
  private final AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private int step;
  private Timer waitTimer;
  /** Creates a new CollectAlgaeFromReef. */
  public CollectAlgaeFromReef(AlgaeManipulatorSubsystem aManipulatorSubsystem) {
    algaeManipulatorSubsystem = aManipulatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeManipulatorSubsystem);
    this.waitTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.step = 0;
    this.waitTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.algaeManipulatorSubsystem.intakeAlgae(0.5);
    switch(this.step) {
      case 0:
        this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(20));
        if(this.algaeManipulatorSubsystem.atTarget()) {
          this.step += 1;
          this.waitTimer.reset();
          this.waitTimer.start();
        }
        break;
      case 1:
        if(this.waitTimer.get() > 1)
          this.algaeManipulatorSubsystem.outtakeCoral();

        if(this.waitTimer.get() > 1.5) {
          this.step += 1;
          this.waitTimer.stop();
        }
        break;
      case 2:
        this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(90));
        if(this.algaeManipulatorSubsystem.atTarget()) {
          this.step += 1;
        }
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
    return this.step > 2;
  }
}
