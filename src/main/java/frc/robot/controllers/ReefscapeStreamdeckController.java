package frc.robot.controllers;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.FieldConstants;

public class ReefscapeStreamdeckController extends StreamdeckController {
    public final Trigger reefOne, reefTwo, reefThree, reefFour, reefFive, reefSix;
    public final Trigger manualNet, manualLevelThree, manualLevelTwo, manualFloor, tiltForward, tiltBack;
    public final Trigger shiftLeft, shiftForward, shiftRight, shiftBack, shifting;
    public final Trigger algaeIntake, algaeOuttake, coralIntake, coralOuttake;
    public final Trigger selectLeft, selectRight;
    public final Trigger rightSide, leftSide;
    public final Trigger autoProcessor, autoLevelOne, autoLevelTwo, autoLevelThree, autoLevelFour, autoNet;
    public final Trigger reset;
    public final Trigger floorCollect, coralLoading;
    public final Trigger semiCoralTrough, semiCoralL2, semiCoralL3, semiCoralL4;
    public final Trigger algaeL1, semiAlgaeL1WithCoral, algaeL2, semiAlgaeNet, semiAlgaeProcessor;

    public enum TargetReef {
        ReefOne,
        ReefTwo,
        ReefThree,
        ReefFour,
        ReefFive,
        ReefSix,
        NONE
    }

    private TargetReef targetReef = TargetReef.NONE;

    public ReefscapeStreamdeckController(ControlScheme defaultControlScheme) {
        super(defaultControlScheme);

        shiftLeft = new Trigger(this.getButton("Shift Left"));
        shiftForward = new Trigger(this.getButton("Shift Forward"));
        shiftRight = new Trigger(this.getButton("Shift Right"));
        shiftBack = new Trigger(this.getButton("Shift Back"));
        shifting = new Trigger(() -> shiftLeft.getAsBoolean() || shiftForward.getAsBoolean()
                || shiftRight.getAsBoolean() || shiftBack.getAsBoolean());

        algaeIntake = new Trigger(this.getButton("Algae Intake"));
        algaeOuttake = new Trigger(this.getButton("Algae Outtake"));
        coralIntake = new Trigger(this.getButton("Coral Intake"));
        coralOuttake = new Trigger(this.getButton("Coral Outtake"));

        selectLeft = new Trigger(this.getButton("Select Left"));
        selectRight = new Trigger(this.getButton("Select Right"));

        rightSide = new Trigger(this.getButton("Right Side"));
        leftSide = new Trigger(this.getButton("Left Side"));

        autoProcessor = new Trigger(this.getButton("Processor"));
        autoLevelOne = new Trigger(this.getButton("Level 1"));
        autoLevelTwo = new Trigger(this.getButton("Level 2"));
        autoLevelThree = new Trigger(this.getButton("Level 3"));
        autoLevelFour = new Trigger(this.getButton("Level 4"));
        autoNet = new Trigger(this.getButton("Net"));

        reefOne = new Trigger(this.getButton("Back Middle"));
        reefTwo = new Trigger(this.getButton("Back Right"));
        reefThree = new Trigger(this.getButton("Front Right"));
        reefFour = new Trigger(this.getButton("Front Middle"));
        reefFive = new Trigger(this.getButton("Front Left"));
        reefSix = new Trigger(this.getButton("Back Left"));
        reset = new Trigger(this.getButton("Reset"));

        manualFloor = new Trigger(this.getButton("Man Floor"));
        manualLevelTwo = new Trigger(this.getButton("Man Level 2"));
        manualLevelThree = new Trigger(this.getButton("Man Level 3"));
        manualNet = new Trigger(this.getButton("Man Net"));

        tiltBack = new Trigger(this.getButton("Tilt Back"));
        tiltForward = new Trigger(this.getButton("Tilt Forward"));

        Trigger hasReefSelected = new Trigger(() -> {
            return reefOne.getAsBoolean() || reefTwo.getAsBoolean() || reefThree.getAsBoolean()
                    || reefFour.getAsBoolean() || reefFive.getAsBoolean() || reefSix.getAsBoolean();
        });
        reefOne.onTrue(Commands.runOnce(() -> this.targetReef = TargetReef.ReefOne));
        reefTwo.onTrue(Commands.runOnce(() -> this.targetReef = TargetReef.ReefTwo));
        reefThree.onTrue(Commands.runOnce(() -> this.targetReef = TargetReef.ReefThree));
        reefFour.onTrue(Commands.runOnce(() -> this.targetReef = TargetReef.ReefFour));
        reefFive.onTrue(Commands.runOnce(() -> this.targetReef = TargetReef.ReefFive));
        reefSix.onTrue(Commands.runOnce(() -> this.targetReef = TargetReef.ReefSix));
        hasReefSelected.onFalse(Commands.runOnce(() -> this.targetReef = TargetReef.NONE));

        floorCollect = new Trigger(this.getButton("Floor Collect"));
        coralLoading = new Trigger(this.getButton("Coral Loading"));

        semiCoralTrough = new Trigger(this.getButton("Coral Trough"));
        semiCoralL2 = new Trigger(this.getButton("Coral L2"));
        semiCoralL3 = new Trigger(this.getButton("Coral L3"));
        semiCoralL4 = new Trigger(this.getButton("Coral L4"));

        algaeL1 = new Trigger(this.getButton("Algae L1"));
        semiAlgaeL1WithCoral = new Trigger(this.getButton("L3 w/ Algae"));
        algaeL2 = new Trigger(this.getButton("Algae L2"));
        semiAlgaeNet = new Trigger(this.getButton("Algae Net"));
        semiAlgaeProcessor = new Trigger(this.getButton("Algae Processor"));
    }

    public TargetReef getTargetReef() {
        return this.targetReef;
    }

    public Optional<Pose2d> getTargetReefPosition(Alliance currentAlliance, boolean isLeft) {
        Pose2d target = null;
        if (isLeft) { // left side
            switch (getTargetReef()) {
                case ReefOne:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.LeftReef.REEF_ONE
                            : FieldConstants.BlueSide.LeftReef.REEF_ONE);
                            break;
                case ReefTwo:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.LeftReef.REEF_TWO
                            : FieldConstants.BlueSide.LeftReef.REEF_TWO);
                            break;
                case ReefThree:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.LeftReef.REEF_THREE
                            : FieldConstants.BlueSide.LeftReef.REEF_THREE);
                            break;
                case ReefFour:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.LeftReef.REEF_FOUR
                            : FieldConstants.BlueSide.LeftReef.REEF_FOUR);
                            break;
                case ReefFive:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.LeftReef.REEF_FIVE
                            : FieldConstants.BlueSide.LeftReef.REEF_FIVE);
                            break;
                case ReefSix:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.LeftReef.REEF_SIX
                            : FieldConstants.BlueSide.LeftReef.REEF_SIX);
                            break;
                case NONE:
                    target = null;
            }
        } else {
            switch (getTargetReef()) {
                case ReefOne:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.RightReef.REEF_ONE
                            : FieldConstants.BlueSide.RightReef.REEF_ONE);
                            break;
                case ReefTwo:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.RightReef.REEF_TWO
                            : FieldConstants.BlueSide.RightReef.REEF_TWO);
                            break;
                case ReefThree:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.RightReef.REEF_THREE
                            : FieldConstants.BlueSide.RightReef.REEF_THREE);
                            break;
                case ReefFour:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.RightReef.REEF_FOUR
                            : FieldConstants.BlueSide.RightReef.REEF_FOUR);
                            break;
                case ReefFive:
                    target = (currentAlliance == Alliance.Red ? FieldConstants.RedSide.RightReef.REEF_FIVE
                            : FieldConstants.BlueSide.RightReef.REEF_FIVE);
                            break;
                case ReefSix:
                    target = currentAlliance == Alliance.Red ? FieldConstants.RedSide.RightReef.REEF_SIX
                            : FieldConstants.BlueSide.RightReef.REEF_SIX;
                            break;
                case NONE:
                    target = null;
            }
        }

        if (target != null)
            return Optional.of(target);
        else
            return Optional.empty();
    }

    public double getXShiftSpeed() {
        if (this.shiftLeft.getAsBoolean())
            return 0.2;
        if (this.shiftRight.getAsBoolean())
            return -0.2;
        return 0;
    }

    public double getYShiftSpeed() {
        if (this.shiftBack.getAsBoolean())
            return -0.75;
        if (this.shiftForward.getAsBoolean())
            return 0.75;
        return 0;
    }

}
