// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

// Example code, TODO: remove before competition season begins
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.example.ExampleCommand;

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

  private ExampleSubsystem exampleSubsystem;
  private ExampleCommand exampleCommand;

  private ClimberSubsystem climberSubsystem;
  private ClimberControlCommand climberControlCommand;

  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorDebuggingControlCommand elevatorControlCommand;

  private AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private AlgaeManipulatorCommand algaeManipulatorCommand;

  private CommandXboxController driverController = new CommandXboxController(
      ControllerConstants.DRIVER_CONTROLLER_PORT);
  private CommandXboxController operatorController = new CommandXboxController(
      ControllerConstants.OPERATOR_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    initializeSwerveSubsystem();
    initializeClimberSubsystem();
    initializeElevatorSubsystem();
    initializeAlgaeManipulatorSubsystem();

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

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(driverController::getRightX,
            driverController::getRightY)
        .headingWhile(true);

    @SuppressWarnings("unused")
    Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    @SuppressWarnings("unused")
    Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    this.swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  private void initializeExampleSubsystem() {
    this.exampleSubsystem = new ExampleSubsystem();
    this.exampleCommand = new ExampleCommand(exampleSubsystem, operatorController);
    this.exampleSubsystem.setDefaultCommand(exampleCommand);

    // Example subsystem can be controlled without having a dedicated control
    // command by using triggers
    operatorController.x().onTrue(exampleSubsystem.runOnce(() -> { // Schedule a one-time command to set the target
                                                                   // position to 700 when the x button is pressed
      exampleSubsystem.setTargetPosition(700);
    }));

    operatorController.y().onTrue(exampleSubsystem.runOnce(() -> { // Schedule a one-time command to set the target
                                                                   // position to 300 when the y button is pressed
      exampleSubsystem.setTargetPosition(300);
    }));
  }

  private void initializeClimberSubsystem() {
    this.climberSubsystem = new ClimberSubsystem();
    this.climberControlCommand = new ClimberControlCommand(climberSubsystem, driverController);
    this.climberSubsystem.setDefaultCommand(climberControlCommand);
  }

  private void initializeElevatorSubsystem() {
    this.elevatorSubsystem = new ElevatorSubsystem();
    // Elevator is now controlled via triggers, a full command is not needed
    if (this.elevatorSubsystem.debugging) {
      this.elevatorSubsystem
          .setDefaultCommand(new ElevatorDebuggingControlCommand(elevatorSubsystem, operatorController));
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

    this.operatorController.leftBumper()
        .onTrue(algaeManipulatorSubsystem.runOnce(() -> algaeManipulatorSubsystem.setIsPIDControlled(false)));
    this.operatorController.leftBumper()
        .onFalse(algaeManipulatorSubsystem.runOnce(() -> algaeManipulatorSubsystem.setIsPIDControlled(true)));
  }

  private void initializeMultisystemCommands() {
  }

  private void initializeNamedCommands() {
    NamedCommands.registerCommand("exampleCommand",
        Commands.runOnce(() -> System.out.println("Example named command has been run!")));

    // Utility Commands
    Command runAlgaeIntake = this.algaeManipulatorSubsystem.run(() -> {
      algaeManipulatorSubsystem.moveLowerWheelMotorRaw(0.5);
      algaeManipulatorSubsystem.moveUpperWheelMotorRaw(-0.5);
    });

    Command runAlgaeOuttake = this.algaeManipulatorSubsystem.run(() -> {
      algaeManipulatorSubsystem.moveLowerWheelMotorRaw(-0.5);
      algaeManipulatorSubsystem.moveUpperWheelMotorRaw(0.5);
    });

    Command runCoralOuttake = this.algaeManipulatorSubsystem.run(() -> {
      algaeManipulatorSubsystem.moveCoralMotorRaw(1);
    });

    Command runCoralIntake = this.algaeManipulatorSubsystem.run(() -> {
      algaeManipulatorSubsystem.moveCoralMotorRaw(-1);
    });

    Command tiltManipulatorForward = algaeManipulatorSubsystem.tiltToAngle(Angle.ofBaseUnits(0, Units.Degrees));
    Command tiltManipulatorBack = algaeManipulatorSubsystem.tiltToAngle(Angle.ofBaseUnits(90, Units.Degrees));


    // Collection Commands
    NamedCommands.registerCommand("CollectAlgaeFromReefL2", new SequentialCommandGroup(
        elevatorSubsystem.moveElevatorToLevel(Level.ALGAE_L1),
        new ParallelRaceGroup(
            runAlgaeIntake,
            new SequentialCommandGroup(
                tiltManipulatorForward,
                new ParallelRaceGroup(
                    runCoralOuttake,
                    new WaitCommand(0.25)),
                tiltManipulatorBack)),
        this.elevatorSubsystem.runOnce(() -> this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR))));

    NamedCommands.registerCommand("CollectAlgaeFromReefL3", new SequentialCommandGroup(
        elevatorSubsystem.moveElevatorToLevel(Level.ALGAE_L2),
        new ParallelRaceGroup(
            runAlgaeIntake,
            new SequentialCommandGroup(
                tiltManipulatorForward,
                new ParallelRaceGroup(
                    runCoralOuttake,
                    new WaitCommand(0.25)),
                tiltManipulatorBack)),
        this.elevatorSubsystem.runOnce(() -> this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR))));

    NamedCommands.registerCommand("CollectAlgaeFromCoralMark", new SequentialCommandGroup(
        this.elevatorSubsystem.moveElevatorToLevel(Level.FLOOR),
        new ParallelRaceGroup(
            runAlgaeIntake,
            new SequentialCommandGroup(
                tiltManipulatorForward,
                new WaitCommand(0.25),
                tiltManipulatorBack))));

    // Scoring Commands
    NamedCommands.registerCommand("ScoreInProcessor", new SequentialCommandGroup(
      this.elevatorSubsystem.moveElevatorToLevel(Level.FLOOR),
      tiltManipulatorForward,
      new ParallelRaceGroup(
        runAlgaeOuttake,
        new WaitCommand(0.25)
      ),
      tiltManipulatorBack
    ));
    NamedCommands.registerCommand("ScoreOnL2", new SequentialCommandGroup(
      this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L2),
      tiltManipulatorForward,
      new ParallelRaceGroup(
        runCoralOuttake,
        new WaitCommand(0.25)
      ),
      tiltManipulatorBack
    ));
    NamedCommands.registerCommand("ScoreOnL3", new SequentialCommandGroup(
      this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L3),
      tiltManipulatorForward,
      new ParallelRaceGroup(
        runCoralOuttake,
        new WaitCommand(0.25)
      ),
      tiltManipulatorBack
    ));
    NamedCommands.registerCommand("ScoreOnNet", new SequentialCommandGroup(
      this.elevatorSubsystem.moveElevatorToLevel(Level.NET),
      tiltManipulatorForward,
      new ParallelRaceGroup(
        runAlgaeOuttake,
        new WaitCommand(0.25)
      ),
      tiltManipulatorBack
    ));

    // Elevator Commands
    NamedCommands.registerCommand("MoveToFloor", this.elevatorSubsystem.moveElevatorToLevel(Level.FLOOR));
    NamedCommands.registerCommand("MoveToL2", this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L2));
    NamedCommands.registerCommand("MoveToL3", this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L3));
    NamedCommands.registerCommand("MoveToL4", this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L4));
    
    // Algae Manipulator Commands
    NamedCommands.registerCommand("RunAlgaeIntake", runAlgaeIntake);
    NamedCommands.registerCommand("RunAlgaeOuttake", runAlgaeOuttake);
    NamedCommands.registerCommand("RunCoralIntake", runCoralIntake);
    NamedCommands.registerCommand("RunCoralOuttake", runCoralOuttake);
    NamedCommands.registerCommand("TiltManipulatorForward", tiltManipulatorForward);
    NamedCommands.registerCommand("TiltManipulatorBack", tiltManipulatorBack);

  }
}
