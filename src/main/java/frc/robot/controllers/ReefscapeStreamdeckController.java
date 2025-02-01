package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ReefscapeStreamdeckController extends StreamdeckController{
    private Trigger reefOne, reefTwo, reefThree, reefFour, reefFive, reefSix;
    private Trigger shiftLeft, shiftForward, shiftRight, shiftBack;
    private Trigger algaeIntake, algaeOuttake, coralIntake, coralOuttake;

    
    public ReefscapeStreamdeckController() {
        super();
        initializeTriggers();
    }

    private void initializeTriggers() {
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

        algaeIntake = new Trigger(this.getButton("Algae Intake"));
        algaeOuttake = new Trigger(this.getButton("Algae Outtake"));
        coralIntake = new Trigger(this.getButton("Coral Intake"));
        coralOuttake = new Trigger(this.getButton("Coral Outtake"));
    }

}
