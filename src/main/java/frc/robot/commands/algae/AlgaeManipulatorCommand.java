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
    if(!this.operatorController.isConnected()){
      // If no operator controller, this command should just tell the coral and algae to stop running
      this.algaeManipulatorSubsystem.holdAlgae();
      this.algaeManipulatorSubsystem.holdCoral();
      return;
    } 
    // This is all control for the operator controller, if it's not connected, don't run it
    
    double outtakeSpeed = this.operatorController.getRightTriggerAxis();
    double intakeSpeed = this.operatorController.getLeftTriggerAxis();

    if (this.operatorController.rightTrigger(0.2).getAsBoolean()) {
      this.algaeManipulatorSubsystem.outtakeAlgae(outtakeSpeed);
    } 
    else if (this.operatorController.leftTrigger(0.2).getAsBoolean()) {
      this.algaeManipulatorSubsystem.intakeAlgae(intakeSpeed);
    } 
    else if (this.operatorController.start().getAsBoolean()) {
      this.algaeManipulatorSubsystem.intakeAlgae(0.2);   
    } 
    else {
      this.algaeManipulatorSubsystem.holdAlgae();
    }

    if (this.operatorController.pov(90).getAsBoolean()) {
      this.algaeManipulatorSubsystem.outtakeCoral();
    } else if (this.operatorController.pov(270).getAsBoolean()) {
      this.algaeManipulatorSubsystem.intakeCoral();
    } else {
      this.algaeManipulatorSubsystem.holdCoral();
    }

    this.algaeManipulatorSubsystem.setOperatorRequestedSpeed(this.operatorController.getLeftY() / 2);
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
