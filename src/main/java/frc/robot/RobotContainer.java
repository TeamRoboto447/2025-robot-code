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

  private CommandXboxController driverController = new CommandXboxController(
      ControllerConstants.DRIVER_CONTROLLER_PORT);
  private ReefscapeStreamdeckController operatorStreamdeck = new ReefscapeStreamdeckController();

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    initializeSwerveSubsystem();
    // initializeStreamdeckBasedControls();

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

  @SuppressWarnings("unused")
  private void initializeStreamdeckBasedControls() {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> operatorStreamdeck.getYShiftSpeed(),
      () -> operatorStreamdeck.getXShiftSpeed())
      .withControllerRotationAxis(() -> 0)
      .deadband(0)
      .scaleTranslation(0.8);

    Command operatorShifting = swerveSubsystem.drive(driveAngularVelocity);
    this.operatorStreamdeck.shifting.whileTrue(operatorShifting);
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
