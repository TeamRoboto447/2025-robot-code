// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multisystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorSubsystemConstants.Level;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualCoralL2 extends Command {

  private final AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final SwerveSubsystem swerveSubsystem;

  private Timer waitTimer;
  private Trigger shiftBackTrigger;
  private Command shiftBackCommand;
  private boolean shiftBack = false;
  private int step;

  /** Creates a new ManualCoralL2. */
  public ManualCoralL2(AlgaeManipulatorSubsystem amSubsystem, ElevatorSubsystem eSubsystem, SwerveSubsystem sSubsystem) {
    this.algaeManipulatorSubsystem = amSubsystem;
    this.elevatorSubsystem = eSubsystem;
    this.swerveSubsystem = sSubsystem;

    addRequirements(amSubsystem, eSubsystem);
    
    this.waitTimer = new Timer();

    // Shifting setup
    this.shiftBackCommand = this.swerveSubsystem
        .drive(SwerveInputStream.of(this.swerveSubsystem.getSwerveDrive(), () -> -0.3, () -> 0) // -0.3
            .withControllerRotationAxis(() -> 0)
            .deadband(0)
            .scaleTranslation(0.8));
    this.shiftBackTrigger = new Trigger(() -> this.shiftBack);
    this.shiftBackTrigger.whileTrue(shiftBackCommand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.elevatorSubsystem.setElevatorTargetHeight(Level.CORAL_L2);
    this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(45));

    this.shiftBack = true;
    this.step = 0;
    this.waitTimer.reset();
    this.waitTimer.start();
    this.algaeManipulatorSubsystem.holdCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (this.algaeManipulatorSubsystem.atTarget() && this.elevatorSubsystem.atTarget()) {
    //   this.algaeManipulatorSubsystem.outtakeCoral();
    // } else {
    //   this.algaeManipulatorSubsystem.holdCoral();
    // }
    switch (this.step) {
      case 0:
        if (waitTimer.get() > 0.2) {
          this.step += 1;
          this.shiftBack = false;
        }
        break;
      case 1:
        if (this.algaeManipulatorSubsystem.atTarget() && this.elevatorSubsystem.atTarget()) {
          this.algaeManipulatorSubsystem.outtakeCoral();
          this.step += 1;
          this.waitTimer.reset();
          this.waitTimer.start();
        }
        break;
      case 2:
        if (this.waitTimer.get() > 0.5)
          this.step += 1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shiftBack = false;
    this.step = 0;
    this.waitTimer.stop();
    this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(85));
    this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.step > 2;
  }
}
