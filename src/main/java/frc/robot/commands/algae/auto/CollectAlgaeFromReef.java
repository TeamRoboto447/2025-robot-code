// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae.auto;

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.step = 0;
    this.waitTimer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.algaeManipulatorSubsystem.intakeAlgae(0.5);
    switch(this.step) {
      case 0:
        this.algaeManipulatorSubsystem.setManipulatorAngle(Angle.ofBaseUnits(0, Units.Degrees));
        if(this.algaeManipulatorSubsystem.atTarget()) {
          this.step += 1;
          this.waitTimer.reset();
          this.waitTimer.start();
        }
        break;
      case 1:
        this.algaeManipulatorSubsystem.moveCoralMotorRaw(-1);
        if(this.waitTimer.get() > 0.25) {
          this.step += 1;
          this.waitTimer.stop();
        }
      case 2:
        this.algaeManipulatorSubsystem.setManipulatorAngle(Angle.ofBaseUnits(90, Units.Degrees));
        if(this.algaeManipulatorSubsystem.atTarget()) {
          this.step += 1;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.step > 2;
  }
}
