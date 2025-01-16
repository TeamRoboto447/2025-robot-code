// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/** Add your docs here. */
public class ControllerDeadbandHandling {
    public static double driverGetX(CommandJoystick driveController) {
        if(Math.abs(driveController.getX()) > Constants.DriverConstants.X_DEADBAND)
            return driveController.getX();
        else return 0;
    }

    public static double driverGetY(CommandJoystick driveController) {
        if(Math.abs(driveController.getY()) > Constants.DriverConstants.Y_DEADBAND)
            return driveController.getY();
        else return 0;
    }

    public static double driverGetZ(CommandJoystick driveController) {
        if(Math.abs(driveController.getZ()) > Constants.DriverConstants.Z_DEADBAND)
            return driveController.getZ();
        else return 0;
    }

    public static double operatorGetLeftX(CommandXboxController driveController) {
        if(Math.abs(driveController.getLeftX()) > Constants.OperatorConstants.DEADBAND)
            return driveController.getLeftX();
        else return 0;
    }

    public static double operatorGetLeftY(CommandXboxController driveController) {
        if(Math.abs(driveController.getLeftY()) > Constants.OperatorConstants.DEADBAND)
            return driveController.getLeftY();
        else return 0;
    }

    public static double operatorGetRightX(CommandXboxController driveController) {
        if(Math.abs(driveController.getRightX()) > Constants.OperatorConstants.DEADBAND)
            return driveController.getRightX();
        else return 0;
    }

    public static double operatorGetRightY(CommandXboxController driveController) {
        if(Math.abs(driveController.getRightY()) > Constants.OperatorConstants.DEADBAND)
            return driveController.getRightY();
        else return 0;
    }
}
