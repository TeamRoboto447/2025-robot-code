package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandOverrides {
    public static Command addDriverOverride(Command command, CommandXboxController driverControl) {
        return new ParallelRaceGroup(
                command,
                new FunctionalCommand(() -> {
                }, () -> {
                }, inturrupted -> {
                }, () -> {
                    return (Math.abs(driverControl.getLeftX()) > 0.2 || Math.abs(driverControl.getLeftY()) > 0.2
                            || Math.abs(driverControl.getRightX()) > 0.2);
                }));
    }
}
