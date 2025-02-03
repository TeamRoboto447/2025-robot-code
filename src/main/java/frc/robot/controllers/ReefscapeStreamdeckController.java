package frc.robot.controllers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ReefscapeStreamdeckController extends StreamdeckController{
    public final Trigger reefOne, reefTwo, reefThree, reefFour, reefFive, reefSix;
    public final Trigger shiftLeft, shiftForward, shiftRight, shiftBack, shifting;
    public final Trigger algaeIntake, algaeOuttake, coralIntake, coralOuttake;
    public final Trigger selectLeft, selectRight;
    public final Trigger processor, floor, levelTwo, levelThree, levelFour, net;

    
    public ReefscapeStreamdeckController() {
        super();

        reefOne = new Trigger(this.getButton("Reef One"));
        reefTwo = new Trigger(this.getButton("Reef Two"));
        reefThree = new Trigger(this.getButton("Reef Three"));
        reefFour = new Trigger(this.getButton("Reef Four"));
        reefFive = new Trigger(this.getButton("Reef Five"));
        reefSix = new Trigger(this.getButton("Reef Six"));

        shiftLeft = new Trigger(this.getButton("Shift Left"));
        shiftForward = new Trigger(this.getButton("Shift Forward"));
        shiftRight = new Trigger(this.getButton("Shift Right"));
        shiftBack = new Trigger(this.getButton("Shift Back"));
        shifting = new Trigger(() -> shiftLeft.getAsBoolean() || shiftForward.getAsBoolean() || shiftRight.getAsBoolean() || shiftBack.getAsBoolean());

        algaeIntake = new Trigger(this.getButton("Algae Intake"));
        algaeOuttake = new Trigger(this.getButton("Algae Outtake"));
        coralIntake = new Trigger(this.getButton("Coral Intake"));
        coralOuttake = new Trigger(this.getButton("Coral Outtake"));

        selectLeft = new Trigger(this.getButton("Select Left"));
        selectRight = new Trigger(this.getButton("Select Right"));

        processor = new Trigger(this.getButton("Processor"));
        floor = new Trigger(this.getButton("Floor"));
        levelTwo = new Trigger(this.getButton("Level 2"));
        levelThree = new Trigger(this.getButton("Level 3"));
        levelFour = new Trigger(this.getButton("Level 4"));
        net = new Trigger(this.getButton("Net"));
    }

    public double getXShiftSpeed() {
        if(this.shiftLeft.getAsBoolean())
            return -0.25;
            if(this.shiftRight.getAsBoolean())
            return 0.25;
        return 0;
    }

    public double getYShiftSpeed() {
        if(this.shiftBack.getAsBoolean())
            return -0.25;
        if(this.shiftForward.getAsBoolean())
            return 0.25;
        return 0;
    }

}
