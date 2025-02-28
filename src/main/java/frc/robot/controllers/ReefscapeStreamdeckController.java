package frc.robot.controllers;

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

    public ReefscapeStreamdeckController() {
        super();

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

        autoProcessor = new Trigger(this.getButton("Processor"));
        autoLevelOne = new Trigger(this.getButton("Level 1"));
        autoLevelTwo = new Trigger(this.getButton("Level 2"));
        autoLevelThree = new Trigger(this.getButton("Level 3"));
        autoLevelFour = new Trigger(this.getButton("Level 4"));
        autoNet = new Trigger(this.getButton("Net"));

        reefOne = new Trigger(this.getButton("Reef One"));
        reefTwo = new Trigger(this.getButton("Reef Two"));
        reefThree = new Trigger(this.getButton("Reef Three"));
        reefFour = new Trigger(this.getButton("Reef Four"));
        reefFive = new Trigger(this.getButton("Reef Five"));
        reefSix = new Trigger(this.getButton("Reef Six"));
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
        reefOne.onTrue(Commands.run(() -> this.targetReef = TargetReef.ReefOne));
        reefTwo.onTrue(Commands.run(() -> this.targetReef = TargetReef.ReefTwo));
        reefThree.onTrue(Commands.run(() -> this.targetReef = TargetReef.ReefThree));
        reefFour.onTrue(Commands.run(() -> this.targetReef = TargetReef.ReefFour));
        reefFive.onTrue(Commands.run(() -> this.targetReef = TargetReef.ReefFive));
        reefSix.onTrue(Commands.run(() -> this.targetReef = TargetReef.ReefSix));
        hasReefSelected.onFalse(Commands.run(() -> this.targetReef = TargetReef.NONE));

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

    public Pose2d getTargetReefPosition(Alliance currentAlliance) {
        switch (this.targetReef) {
            case ReefOne:
                return currentAlliance == Alliance.Red ? FieldConstants.RedSide.REEF_ONE
                        : FieldConstants.BlueSide.REEF_ONE;
            case ReefTwo:
                return currentAlliance == Alliance.Red ? FieldConstants.RedSide.REEF_TWO
                        : FieldConstants.BlueSide.REEF_TWO;
            case ReefThree:
                return currentAlliance == Alliance.Red ? FieldConstants.RedSide.REEF_THREE
                        : FieldConstants.BlueSide.REEF_THREE;
            case ReefFour:
                return currentAlliance == Alliance.Red ? FieldConstants.RedSide.REEF_FOUR
                        : FieldConstants.BlueSide.REEF_FOUR;
            case ReefFive:
                return currentAlliance == Alliance.Red ? FieldConstants.RedSide.REEF_FIVE
                        : FieldConstants.BlueSide.REEF_FIVE;
            case ReefSix:
                return currentAlliance == Alliance.Red ? FieldConstants.RedSide.REEF_SIX
                        : FieldConstants.BlueSide.REEF_SIX;
            default:
                return null;

        }
    }

    public double getXShiftSpeed() {
        if (this.shiftLeft.getAsBoolean())
            return 0.25;
        if (this.shiftRight.getAsBoolean())
            return -0.25;
        return 0;
    }

    public double getYShiftSpeed() {
        if (this.shiftBack.getAsBoolean())
            return -0.25;
        if (this.shiftForward.getAsBoolean())
            return 0.25;
        return 0;
    }

}
