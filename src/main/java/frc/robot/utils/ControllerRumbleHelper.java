// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class ControllerRumbleHelper {
    public static Command rumbleLeft(CommandXboxController controller, double strength, double seconds) {
        return new ParallelRaceGroup(
                rumbleLeft(controller, strength),
                new WaitCommand(seconds));
    }

    public static Command rumbleLeft(CommandXboxController controller, double strength) {
        return new FunctionalCommand(
                () -> {
                    controller.setRumble(RumbleType.kLeftRumble, strength);
                },
                () -> {
                    // Execute
                },
                (interrupted) -> {
                    controller.setRumble(RumbleType.kLeftRumble, 0);
                },
                () -> {
                    // isFinished
                    return false;
                });
    }

    public static Command rumbleRight(CommandXboxController controller, double strength, double seconds) {
        return new ParallelRaceGroup(
                rumbleRight(controller, strength),
                new WaitCommand(seconds));
    }

    public static Command rumbleRight(CommandXboxController controller, double strength) {
        return new FunctionalCommand(
                () -> {
                    controller.setRumble(RumbleType.kRightRumble, strength);
                },
                () -> {
                    // Execute
                },
                (interrupted) -> {
                    controller.setRumble(RumbleType.kRightRumble, 0);
                },
                () -> {
                    // isFinished
                    return false;
                });
    }

    public static Command rumbleBoth(CommandXboxController controller, double strength, double seconds) {
        return new ParallelRaceGroup(
                rumbleBoth(controller, strength),
                new WaitCommand(seconds));
    }

    public static Command rumbleBoth(CommandXboxController controller, double strength) {
        return new FunctionalCommand(
                () -> {
                    controller.setRumble(RumbleType.kBothRumble, strength);
                },
                () -> {
                    // Execute
                },
                (interrupted) -> {
                    controller.setRumble(RumbleType.kBothRumble, 0);
                },
                () -> {
                    // isFinished
                    return false;
                });
    }
}
