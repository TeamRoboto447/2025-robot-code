// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.utils.ControllerRumbleHelper.rumbleLeft;
import static frc.robot.utils.ControllerRumbleHelper.rumbleBoth;
import static frc.robot.utils.ControllerRumbleHelper.rumbleRight;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.algae.AlgaeManipulatorCommand;
import frc.robot.commands.algae.auto.CollectAlgaeFromReef;
import frc.robot.Constants.ElevatorSubsystemConstants.Level;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.climber.ClimberControlCommand;
import frc.robot.commands.elevator.ElevatorDebuggingControlCommand;
import frc.robot.commands.multisystem.ManualAlgaeL1;
import frc.robot.commands.multisystem.AlgaeL2Command;
import frc.robot.commands.multisystem.ManualAlgaeNet;
import frc.robot.commands.multisystem.ManualAlgaeProcessor;
import frc.robot.commands.multisystem.ManualCoralL1;
import frc.robot.commands.multisystem.ManualCoralL2;
import frc.robot.commands.multisystem.ManualCoralL3;
import frc.robot.commands.multisystem.CoralL3AlgaeL1Command;
import frc.robot.commands.multisystem.ManualCoralL4;
import frc.robot.commands.multisystem.ManualCoralPickup;
import frc.robot.commands.multisystem.ManualFloorPickup;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.utils.CommandOverrides;
import swervelib.SwerveInputStream;

import frc.robot.controllers.ReefscapeStreamdeckController;
import frc.robot.controllers.StreamdeckController.ControlScheme;

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

  public AlgaeManipulatorSubsystem algaeManipulatorSubsystem;
  private AlgaeManipulatorCommand algaeManipulatorCommand;

  private ManualFloorPickup manualFloorPickupCommand;
  private ManualCoralPickup manualCoralPickupCommand;
  private ManualCoralL1 manualCoralL1Command;
  private ManualCoralL2 manualCoralL2Command;
  private ManualCoralL3 manualCoralL3Command;
  private CoralL3AlgaeL1Command manualCoralL3AlgaeL1Command;
  private ManualCoralL4 manualCoralL4Command;
  private ManualAlgaeL1 manualAlgaeL1Command;
  private AlgaeL2Command manualAlgaeL2Command;
  private ManualAlgaeNet manualAlgaeNetCommand;
  private ManualAlgaeProcessor manualAlgaeProcessorCommand;

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
    initializeAlgaeManipulatorSubsystem();
    initializeElevatorSubsystem();

    // initializeExampleSubsystem();
    initializeMultisystemCommands();

    initializeNamedCommands();

    // Configure the trigger bindings
    configureMultisystemBindings();
    initializeStreamdeckBasedControls();
    initializeControllerRumbles();

    // Configure the PoseEstimatorSubsystem
    new PoseEstimatorSubsystem(swerveSubsystem);

    // this.driverController.a()
    // .whileTrue(this.algaeManipulatorSubsystem.run(() ->
    // this.algaeManipulatorSubsystem.outtakeAlgae(1)));

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
    Trigger driverShifting = new Trigger(() -> driverController.pov(90).getAsBoolean() || driverController.pov(270).getAsBoolean());
    SwerveInputStream arrowKeyInputStream = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
        () -> driverController.pov(90).getAsBoolean() ? 1 : (driverController.pov(270).getAsBoolean() ? -1 : 0),
        () -> 0)
        .withControllerRotationAxis(() -> -driverController.getRightX() / 2)
        .deadband(DriverConstants.DEADBAND)
        .scaleTranslation(0.8);
    

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
        () -> driverController.getLeftY() * -1,
        () -> driverController.getLeftX() * -1)
        .withControllerRotationAxis(() -> -driverController.getRightX() / 2)
        .deadband(DriverConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    this.swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    driverShifting.whileTrue(swerveSubsystem.drive(arrowKeyInputStream));
  }

  private void initializeStreamdeckBasedControls() {
    // Operator controlled shifting
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
        () -> operatorStreamdeck.getYShiftSpeed(),
        () -> operatorStreamdeck.getXShiftSpeed())
        .withControllerRotationAxis(() -> -driverController.getRightX() / 2)
        .deadband(0)
        .scaleTranslation(0.8);

    Command operatorShifting = swerveSubsystem.drive(driveAngularVelocity);
    this.operatorStreamdeck.shifting.whileTrue(operatorShifting);
    this.operatorStreamdeck.shifting.onTrue(rumbleBoth(driverController, 1, 0.125));

    initializeLegacyStreamdeckControls();

    // Intake
    this.operatorStreamdeck.floorCollect.whileTrue(this.manualFloorPickupCommand);
    this.operatorStreamdeck.coralLoading.whileTrue(this.manualCoralPickupCommand);

    // Semi-Auto Control Scheme
    this.operatorStreamdeck.semiCoralTrough.whileTrue(this.manualCoralL1Command);
    this.operatorStreamdeck.semiCoralL2.onTrue(this.manualCoralL2Command);
    this.operatorStreamdeck.semiCoralL3.onTrue(this.manualCoralL3Command);
    this.operatorStreamdeck.semiCoralL4.onTrue(this.manualCoralL4Command);

    this.operatorStreamdeck.algaeL1.onTrue(this.manualAlgaeL1Command);
    this.operatorStreamdeck.semiAlgaeL1WithCoral.onTrue(this.manualCoralL3AlgaeL1Command);
    this.operatorStreamdeck.algaeL2.onTrue(this.manualAlgaeL2Command);

    this.operatorStreamdeck.semiAlgaeNet.whileTrue(this.manualAlgaeNetCommand);
    this.operatorStreamdeck.semiAlgaeProcessor.whileTrue(this.manualAlgaeProcessorCommand);

    // Fully-Auto Control Scheme
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    Alliance alliance = Alliance.Blue;
    if (allianceOptional.isPresent())
      alliance = allianceOptional.get();

    Pose2d targetNet = alliance == Alliance.Red ? FieldConstants.RedSide.NET : FieldConstants.BlueSide.NET;
    this.operatorStreamdeck.autoNet
        .onTrue(CommandOverrides.addDriverOverride(swerveSubsystem.driveToPose(targetNet), driverController));

    Pose2d targetProc = alliance == Alliance.Red ? FieldConstants.RedSide.PROC : FieldConstants.BlueSide.PROC;
    this.operatorStreamdeck.autoProcessor
        .onTrue(CommandOverrides.addDriverOverride(swerveSubsystem.driveToPose(targetProc), driverController));

    // this.operatorStreamdeck.autoLevelOne.onTrue(CommandOverrides.addDriverOverride(
    //     swerveSubsystem.driveToPose(this.operatorStreamdeck.getTargetReefPosition(alliance)), driverController));
    // this.operatorStreamdeck.autoLevelTwo.onTrue(CommandOverrides.addDriverOverride(
    //     swerveSubsystem.driveToPose(this.operatorStreamdeck.getTargetReefPosition(alliance)), driverController));
    // this.operatorStreamdeck.autoLevelThree.onTrue(CommandOverrides.addDriverOverride(
    //     swerveSubsystem.driveToPose(this.operatorStreamdeck.getTargetReefPosition(alliance)), driverController));
    // this.operatorStreamdeck.autoLevelFour.onTrue(CommandOverrides.addDriverOverride(
    //     swerveSubsystem.driveToPose(this.operatorStreamdeck.getTargetReefPosition(alliance)), driverController));
  }

  private void initializeLegacyStreamdeckControls() {
    this.operatorStreamdeck.reefOne
        .onChange(this.algaeManipulatorSubsystem.runOnce(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            System.out.println("Reef One Status Changed");
        }));

    this.operatorStreamdeck.algaeIntake
        .whileTrue(this.algaeManipulatorSubsystem.run(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.algaeManipulatorSubsystem.intakeAlgae(0.5);
        }));

    this.operatorStreamdeck.algaeOuttake
        .whileTrue(this.algaeManipulatorSubsystem.run(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.algaeManipulatorSubsystem.outtakeAlgae(0.5);
        }));

    this.operatorStreamdeck.coralIntake
        .whileTrue(this.algaeManipulatorSubsystem.run(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.algaeManipulatorSubsystem.intakeCoral();
        }));

    this.operatorStreamdeck.coralOuttake
        .whileTrue(this.algaeManipulatorSubsystem.run(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.algaeManipulatorSubsystem.outtakeCoral();
        }));

    this.operatorStreamdeck.manualNet
        .onTrue(this.elevatorSubsystem.runOnce(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.elevatorSubsystem.setElevatorTargetHeight(Level.NET);
        }));
    this.operatorStreamdeck.manualLevelTwo
        .onTrue(this.elevatorSubsystem.runOnce(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.elevatorSubsystem.setElevatorTargetHeight(Level.ALGAE_L1);
        }));
    this.operatorStreamdeck.manualLevelThree
        .onTrue(this.elevatorSubsystem.runOnce(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.elevatorSubsystem.setElevatorTargetHeight(Level.ALGAE_L2);
        }));
    this.operatorStreamdeck.manualFloor
        .onTrue(this.elevatorSubsystem.runOnce(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR);
        }));

    this.operatorStreamdeck.tiltBack.whileTrue(this.algaeManipulatorSubsystem.run(() -> {
      if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY) {
        this.algaeManipulatorSubsystem.setIsPIDControlled(false);
        this.algaeManipulatorSubsystem.setOperatorRequestedSpeed(0.5);
      }
    }));
    this.operatorStreamdeck.tiltBack
        .onFalse(this.algaeManipulatorSubsystem.runOnce(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.algaeManipulatorSubsystem.setIsPIDControlled(true);
        }));

    this.operatorStreamdeck.tiltForward.whileTrue(this.algaeManipulatorSubsystem.run(() -> {
      if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY) {
        this.algaeManipulatorSubsystem.setIsPIDControlled(false);
        this.algaeManipulatorSubsystem.setOperatorRequestedSpeed(-0.5);
      }
    }));

    this.operatorStreamdeck.tiltForward
        .onFalse(this.algaeManipulatorSubsystem.runOnce(() -> {
          if (this.operatorStreamdeck.getCurrentScheme() == ControlScheme.LEGACY)
            this.algaeManipulatorSubsystem.setIsPIDControlled(true);
        }));
  }

  private void initializeClimberSubsystem() {
    this.climberSubsystem = new ClimberSubsystem();
    this.climberControlCommand = new ClimberControlCommand(climberSubsystem, driverController);
    this.climberSubsystem.setDefaultCommand(climberControlCommand);
  }

  private void initializeElevatorSubsystem() {
    this.elevatorSubsystem = new ElevatorSubsystem(this.algaeManipulatorSubsystem);
    // Elevator is now controlled via triggers, a full command is not needed
    if (this.elevatorSubsystem.debugging) {
      this.elevatorSubsystem
          .setDefaultCommand(new ElevatorDebuggingControlCommand(elevatorSubsystem, operatorController));
      return;
    }

    if (!this.operatorController.isConnected())
      return;

    this.operatorController.a().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.increaseLevel()));
    this.operatorController.b().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.decreaseLevel()));
    this.operatorController.y()
        .onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.setElevatorTargetHeight(Level.NET)));
    this.operatorController.x()
        .onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.setElevatorTargetHeight(Level.FLOOR)));

  }

  private void initializeAlgaeManipulatorSubsystem() {
    this.algaeManipulatorSubsystem = new AlgaeManipulatorSubsystem();
    this.algaeManipulatorCommand = new AlgaeManipulatorCommand(algaeManipulatorSubsystem, operatorController);
    this.algaeManipulatorSubsystem.setDefaultCommand(algaeManipulatorCommand);

    if (!this.operatorController.isConnected())
      return;
    Trigger leftYPastDeadzone = new Trigger(() -> Math.abs(this.operatorController.getLeftY()) > 0.5);
    leftYPastDeadzone
        .onTrue(algaeManipulatorSubsystem.runOnce(() -> algaeManipulatorSubsystem.setIsPIDControlled(false)));
    leftYPastDeadzone
        .onFalse(algaeManipulatorSubsystem.runOnce(() -> algaeManipulatorSubsystem.setIsPIDControlled(true)));
  }

  private void initializeMultisystemCommands() {
    this.manualFloorPickupCommand = new ManualFloorPickup(algaeManipulatorSubsystem, elevatorSubsystem);
    this.manualCoralPickupCommand = new ManualCoralPickup(algaeManipulatorSubsystem, elevatorSubsystem);
    this.manualCoralL1Command = new ManualCoralL1(algaeManipulatorSubsystem, elevatorSubsystem);
    this.manualCoralL2Command = new ManualCoralL2(algaeManipulatorSubsystem, elevatorSubsystem, swerveSubsystem);
    this.manualCoralL3Command = new ManualCoralL3(algaeManipulatorSubsystem, elevatorSubsystem);
    this.manualCoralL4Command = new ManualCoralL4(algaeManipulatorSubsystem, elevatorSubsystem, swerveSubsystem);

    this.manualAlgaeL1Command = new ManualAlgaeL1(algaeManipulatorSubsystem, elevatorSubsystem);
    this.manualAlgaeL2Command = new AlgaeL2Command(algaeManipulatorSubsystem, elevatorSubsystem);

    this.manualCoralL3AlgaeL1Command = new CoralL3AlgaeL1Command(algaeManipulatorSubsystem, elevatorSubsystem);

    this.manualAlgaeNetCommand = new ManualAlgaeNet(algaeManipulatorSubsystem, elevatorSubsystem);
    this.manualAlgaeProcessorCommand = new ManualAlgaeProcessor(algaeManipulatorSubsystem, elevatorSubsystem);

    AllianceStationID allianceStationID = DriverStation.getRawAllianceStation();
    Pose2d cagePosition = null;
    switch (allianceStationID) {
      case Blue1:
        cagePosition = FieldConstants.BlueSide.CAGE_ONE;
        break;
      case Blue2:
        cagePosition = FieldConstants.BlueSide.CAGE_TWO;
        break;
      case Blue3:
        cagePosition = FieldConstants.BlueSide.CAGE_THREE;
        break;
      case Red1:
        cagePosition = FieldConstants.RedSide.CAGE_ONE;
        break;
      case Red2:
        cagePosition = FieldConstants.RedSide.CAGE_TWO;
        break;
      case Red3:
        cagePosition = FieldConstants.RedSide.CAGE_THREE;
        break;
      default:
        break;

    }
    this.driverController.a()
        .onTrue(CommandOverrides.addDriverOverride(swerveSubsystem.driveToPose(cagePosition), driverController));
  }

  private void initializeNamedCommands() {
    // Collection Commands
    NamedCommands.registerCommand("CollectAlgaeFromReefL2",
        new CoralL3AlgaeL1Command(algaeManipulatorSubsystem, elevatorSubsystem));

    NamedCommands.registerCommand("CollectAlgaeFromReefL3",
        new AlgaeL2Command(algaeManipulatorSubsystem, elevatorSubsystem));

    NamedCommands.registerCommand("CollectAlgaeFromCoralMark", new SequentialCommandGroup(
        this.elevatorSubsystem.moveElevatorToLevel(Level.FLOOR),
        new CollectAlgaeFromReef(algaeManipulatorSubsystem)));

    // Scoring Commands
    NamedCommands.registerCommand("ScoreInProcessor", new ParallelRaceGroup(
        new ManualAlgaeProcessor(algaeManipulatorSubsystem, elevatorSubsystem),
        new WaitCommand(0.5)));

    NamedCommands.registerCommand("ScoreOnL2",
        new SequentialCommandGroup(this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L2),
            algaeManipulatorSubsystem.tiltToAngle(Degrees.of(0)),
            new ParallelRaceGroup(
                this.algaeManipulatorSubsystem.run(() -> {
                  algaeManipulatorSubsystem.outtakeCoral();
                }),
                new WaitCommand(0.25)),
            algaeManipulatorSubsystem.tiltToAngle(Degrees.of(90)),
            this.elevatorSubsystem.moveElevatorToLevel(Level.FLOOR)));

    NamedCommands.registerCommand("ScoreOnL3", new SequentialCommandGroup(
        this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L3),
        algaeManipulatorSubsystem.tiltToAngle(Degrees.of(0)),
        new ParallelRaceGroup(
            this.algaeManipulatorSubsystem.run(() -> {
              algaeManipulatorSubsystem.outtakeCoral();
            }),
            new WaitCommand(0.25)),
        algaeManipulatorSubsystem.tiltToAngle(Degrees.of(90)),
        this.elevatorSubsystem.moveElevatorToLevel(Level.FLOOR)));

    NamedCommands.registerCommand("ScoreInNet", new SequentialCommandGroup(
        this.elevatorSubsystem.moveElevatorToLevel(Level.NET),
        new ParallelRaceGroup(
            this.algaeManipulatorSubsystem.run(() -> {
              algaeManipulatorSubsystem.outtakeAlgae(0.5);
            }),
            new WaitCommand(0.25)),
        algaeManipulatorSubsystem.tiltToAngle(Degrees.of(90)),
        this.elevatorSubsystem.moveElevatorToLevel(Level.FLOOR)));

    // Elevator Commands
    NamedCommands.registerCommand("MoveToFloor", this.elevatorSubsystem.moveElevatorToLevel(Level.FLOOR));
    NamedCommands.registerCommand("MoveToL2", this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L2));
    NamedCommands.registerCommand("MoveToL3", this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L3));
    NamedCommands.registerCommand("MoveToL4", this.elevatorSubsystem.moveElevatorToLevel(Level.CORAL_L4));

    // Algae Manipulator Commands
    NamedCommands.registerCommand("RunAlgaeIntake", this.algaeManipulatorSubsystem.run(() -> {
      algaeManipulatorSubsystem.intakeAlgae(0.5);
    }));
    NamedCommands.registerCommand("RunAlgaeOuttake", this.algaeManipulatorSubsystem.run(() -> {
      algaeManipulatorSubsystem.outtakeAlgae(0.5);
    }));
    NamedCommands.registerCommand("RunCoralIntake", this.algaeManipulatorSubsystem.run(() -> {
      algaeManipulatorSubsystem.intakeCoral();
    }));
    NamedCommands.registerCommand("RunCoralOuttake", this.algaeManipulatorSubsystem.run(() -> {
      algaeManipulatorSubsystem.outtakeCoral();
    }));
    NamedCommands.registerCommand("TiltForward",
        algaeManipulatorSubsystem.tiltToAngle(Degrees.of(0)));
    NamedCommands.registerCommand("TiltBack",
        algaeManipulatorSubsystem.tiltToAngle(Degrees.of(90)));

  }

  private void initializeControllerRumbles() {
    this.driverController.x().whileTrue(rumbleLeft(driverController, 1));
    this.driverController.b().whileTrue(rumbleRight(driverController, 1));
    this.driverController.y().whileTrue(rumbleBoth(driverController, 1));
    this.driverController.a().onTrue(rumbleBoth(driverController, 1, 2));
  }
}
