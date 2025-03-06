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
public class CoralL4AlgaeL1Command extends Command {

  private final AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final SwerveSubsystem swerveSubsystem;

  private final Trigger shiftBackTrigger;
  private Command shiftBackCommand;
  private boolean shiftBack = false;

  private final Trigger shiftForwardTrigger;
  private Command shiftForwardCommand;
  private boolean shiftForward;

  private int step;
  private Timer waitTimer;

  /** Creates a new ManualCoralL3AlgaeL1. */
  public CoralL4AlgaeL1Command(AlgaeManipulatorSubsystem amSubsystem, ElevatorSubsystem eSubsystem,
      SwerveSubsystem sSubsystem) {
    this.algaeManipulatorSubsystem = amSubsystem;
    this.elevatorSubsystem = eSubsystem;
    this.swerveSubsystem = sSubsystem;

    addRequirements(amSubsystem, eSubsystem);
    this.waitTimer = new Timer();
    this.waitTimer.stop();

    // Shifting setup
    this.shiftBackCommand = this.swerveSubsystem
        .drive(SwerveInputStream.of(this.swerveSubsystem.getSwerveDrive(), () -> -0.3, () -> 0)
            .withControllerRotationAxis(() -> 0)
            .deadband(0)
            .scaleTranslation(0.8));
    this.shiftBackTrigger = new Trigger(() -> this.shiftBack);
    this.shiftBackTrigger.whileTrue(shiftBackCommand);

    this.shiftForwardCommand = this.swerveSubsystem
        .drive(SwerveInputStream.of(this.swerveSubsystem.getSwerveDrive(), () -> 0.3, () -> 0)
            .withControllerRotationAxis(() -> 0)
            .deadband(0)
            .scaleTranslation(0.8));
    this.shiftForwardTrigger = new Trigger(() -> this.shiftForward);
    this.shiftForwardTrigger.whileTrue(shiftForwardCommand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shiftBack = true;

    this.step = 0;
    this.waitTimer.reset();
    this.waitTimer.start();
    this.algaeManipulatorSubsystem.holdCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.algaeManipulatorSubsystem.intakeAlgae(1);
    switch (this.step) {
      case 0:
        if (waitTimer.get() > 0.5) {
          this.step += 1;
        }
        break;
      case 1:
        this.shiftBack = false;
        this.elevatorSubsystem.setElevatorTargetHeight(Level.CORAL_L4);
        this.step += 1;
        break;
      case 2:
        if (this.elevatorSubsystem.atTarget()) {
          this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(20));
          this.step += 1;
        }
        break;
      case 3:
        if (this.algaeManipulatorSubsystem.atTarget()) {
          this.algaeManipulatorSubsystem.outtakeCoral();
          this.waitTimer.reset();
          this.waitTimer.start();
          this.step += 1;
        }
        break;
      case 4:
        if (this.waitTimer.get() > 0.5)
          this.step += 1;
        break;
      case 5:
        this.shiftBack = false;
        this.step = 0;
        this.waitTimer.stop();
        this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(85));
        this.elevatorSubsystem.setElevatorTargetHeight(Level.ALGAE_L1);
        break;
      case 6:
        if (this.elevatorSubsystem.atTarget()) {
          this.shiftForward = true;
          this.waitTimer.reset();
          this.waitTimer.start();
          this.step += 1;
        }
        break;
      case 7:
        if (this.waitTimer.get() > 0.5)
          this.step += 1;
        break;
      case 8:
        this.shiftForward = false;
        this.algaeManipulatorSubsystem.setManipulatorAngle(Degrees.of(10));
        this.waitTimer.reset();
        this.waitTimer.start();
        this.step += 1;
        break;
      case 9:
        if (this.algaeManipulatorSubsystem.atTarget() && this.waitTimer.get() > 2)
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return step > 9;
  }
}
