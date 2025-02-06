// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.algae.AlgaeManipulatorCommand;
import frc.robot.Constants.ElevatorSubsystemConstants.Level;
import frc.robot.commands.climber.ClimberControlCommand;
import frc.robot.commands.elevator.ElevatorDebuggingControlCommand;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import frc.robot.controllers.ReefscapeStreamdeckController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SwerveSubsystem swerveSubsystem;

  private ClimberSubsystem climberSubsystem;
  private ClimberControlCommand climberControlCommand;

  private ElevatorSubsystem elevatorSubsystem;

  private AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private AlgaeManipulatorCommand algaeManipulatorCommand;

  private CommandXboxController driverController = new CommandXboxController(
      ControllerConstants.DRIVER_CONTROLLER_PORT);
  private CommandXboxController operatorController = new CommandXboxController(
      ControllerConstants.OPERATOR_CONTROLLER_PORT);
  private ReefscapeStreamdeckController operatorStreamdeck = new ReefscapeStreamdeckController();

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    initializeSwerveSubsystem();
    initializeClimberSubsystem();
    initializeElevatorSubsystem();
    initializeAlgaeManipulatorSubsystem();
    initializeStreamdeckBasedControls();

    // initializeExampleSubsystem();
    initializeMultisystemCommands();

    initializeNamedCommands();

    // Configure the trigger bindings
    configureMultisystemBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureMultisystemBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  private void initializeSwerveSubsystem() {
    this.swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
        "swerve"));

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> driverController.getLeftY() * -1,
      () -> driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverController.getRightX())
      .deadband(DriverConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

    Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    this.swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  private void initializeStreamdeckBasedControls() {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> operatorStreamdeck.getYShiftSpeed(),
      () -> operatorStreamdeck.getXShiftSpeed())
      .withControllerRotationAxis(() -> 0)
      .deadband(0)
      .scaleTranslation(0.8);

    Command operatorShifting = swerveSubsystem.drive(driveAngularVelocity);
    this.operatorStreamdeck.shifting.whileTrue(operatorShifting);
    this.operatorStreamdeck.reefOne.onChange(this.algaeManipulatorSubsystem.runOnce(() -> System.out.println("Reef One Status Changed")));

    this.operatorStreamdeck.algaeIntake.whileTrue(this.algaeManipulatorSubsystem.run(() -> this.algaeManipulatorSubsystem.intakeAlgae(0.5)));
    this.operatorStreamdeck.algaeOuttake.whileTrue(this.algaeManipulatorSubsystem.run(() -> this.algaeManipulatorSubsystem.outtakeAlgae(0.5)));
    this.operatorStreamdeck.coralIntake.whileTrue(this.algaeManipulatorSubsystem.run(() -> this.algaeManipulatorSubsystem.moveCoralMotorRaw(1)));
    this.operatorStreamdeck.coralOuttake.whileTrue(this.algaeManipulatorSubsystem.run(() -> this.algaeManipulatorSubsystem.moveCoralMotorRaw(-1)));
  }

  private void initializeClimberSubsystem() {
    this.climberSubsystem = new ClimberSubsystem();
    this.climberControlCommand = new ClimberControlCommand(climberSubsystem, driverController);
    this.climberSubsystem.setDefaultCommand(climberControlCommand);
  }

  private void initializeElevatorSubsystem() {
    this.elevatorSubsystem = new ElevatorSubsystem();
    // Elevator is now controlled via triggers, a full command is not needed
    if(this.elevatorSubsystem.debugging) {
      this.elevatorSubsystem.setDefaultCommand(new ElevatorDebuggingControlCommand(elevatorSubsystem, operatorController));
    } else {
      this.operatorController.a().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.increaseLevel()));
      this.operatorController.b().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.decreaseLevel()));
      this.operatorController.y()
          .onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.setElevatorTargetHeight(Level.NET)));
      this.operatorController.x()
          .onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR)));
    }
  }

  private void initializeAlgaeManipulatorSubsystem() {
    this.algaeManipulatorSubsystem = new AlgaeManipulatorSubsystem();
    this.algaeManipulatorCommand = new AlgaeManipulatorCommand(algaeManipulatorSubsystem, operatorController);
    this.algaeManipulatorSubsystem.setDefaultCommand(algaeManipulatorCommand);

    Trigger leftYPastDeadzone = new Trigger(() -> Math.abs(this.operatorController.getLeftY()) > 0.5);
    leftYPastDeadzone.onTrue(algaeManipulatorSubsystem.runOnce(() -> algaeManipulatorSubsystem.setIsPIDControlled(false)));
    leftYPastDeadzone.onFalse(algaeManipulatorSubsystem.runOnce(() -> algaeManipulatorSubsystem.setIsPIDControlled(true)));
  }

  private void initializeMultisystemCommands() {
  }

  private void initializeNamedCommands() {
    // NamedCommands.registerCommand("exampleCommand",
    //     Commands.runOnce(() -> System.out.println("Example named command has been run!")));

    // // Utility Commands
    // Command runAlgaeIntake = this.algaeManipulatorSubsystem.run(() -> {
    //   algaeManipulatorSubsystem.moveLowerWheelMotorRaw(0);
    //   algaeManipulatorSubsystem.moveUpperWheelMotorRaw(0);
    // });

    // NamedCommands.registerCommand("CollectAlgaeFromReef", new SequentialCommandGroup(
    //     new ParallelRaceGroup(
    //         runAlgaeIntake,
    //         new SequentialCommandGroup(
    //             // TODO: Tilt forward command,
    //             new ParallelRaceGroup(
    //                 //TODO: Add Coral Outtake
    //                 new WaitCommand(0.25))
    //         // TODO: Tilt back command
    //         )),
    //     this.elevatorSubsystem.runOnce(() -> this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR))));

    // NamedCommands.registerCommand("CollectAlgaeFromCoralMark", new SequentialCommandGroup(
    //     this.elevatorSubsystem.runOnce(() -> this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR)),
    //     new ParallelRaceGroup(
    //         runAlgaeIntake,
    //         new SequentialCommandGroup(
    //             // TODO: Tilt forward command,
    //             new WaitCommand(0.25)
    //         // TODO: Tilt back command
    //         ))));

    // // Elevator Commands

    // // Algae Manipulator Commands
    // NamedCommands.registerCommand("RunAlgaeIntake", runAlgaeIntake);
  }
}
