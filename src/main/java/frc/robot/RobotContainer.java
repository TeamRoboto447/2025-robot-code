// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.climber.ClimberControlCommand;
import frc.robot.commands.drivebase.AbsoluteSwerveDriveAdv;
import frc.robot.commands.elevator.ElevatorControlCommand;
// import frc.robot.commands.example.ExampleCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SwerveSubsystem swerveSubsystem;
  private AbsoluteSwerveDriveAdv absoluteDriveCommand; // TODO: Fix this command

  // private ExampleSubsystem exampleSubsystem;
  // private ExampleCommand exampleCommand;

  private ClimberSubsystem climberSubsystem;
  private ClimberControlCommand climberControlCommand;

  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorControlCommand elevatorControlCommand;

  private CommandJoystick driverController = new CommandJoystick(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private CommandXboxController operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSwerveSubsystem();
    initializeClimberSubsystem();
    initializeElevatorSubsystem();
   
    // initializeExampleSubsystem();
    initializeMultisystemCommands();

    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  private void initializeSwerveSubsystem() {
    this.swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    // this.absoluteDriveCommand = new AbsoluteSwerveDriveAdv(swerveSubsystem, 
    // () -> this.driverController.getX(), () -> this.driverController.getY(), () -> this.driverController.getZ(), () -> false, () -> false, () -> false, () -> false);
    this.swerveSubsystem.setDefaultCommand(this.swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 0));
  }

  // private void initializeExampleSubsystem() {
  //   this.exampleSubsystem = new ExampleSubsystem();
  //   this.exampleCommand = new ExampleCommand(exampleSubsystem, operatorController);
  //   this.exampleSubsystem.setDefaultCommand(exampleCommand);
  // }
  
  private void initializeClimberSubsystem() {
    this.climberSubsystem = new ClimberSubsystem();
    this.climberControlCommand = new ClimberControlCommand(climberSubsystem, driverController);
    this.climberSubsystem.setDefaultCommand(climberControlCommand);
  }

  private void initializeElevatorSubsystem() {
    this.elevatorSubsystem = new ElevatorSubsystem();
    this.elevatorControlCommand = new ElevatorControlCommand(elevatorSubsystem, operatorController);
    this.elevatorSubsystem.setDefaultCommand(elevatorControlCommand);
  }
  
  private void initializeMultisystemCommands() {
  }
}
