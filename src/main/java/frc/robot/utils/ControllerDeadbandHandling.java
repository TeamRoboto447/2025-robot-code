package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/** Add your docs here. */
public class ControllerDeadbandHandling {

    // Generic deadband function
    public static double applyGenericDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0;
    }

    // Deadband handling pulling the deadband constants from the Constants file
    public static double driverGetLeftX(CommandXboxController driveController) {
        return applyGenericDeadband(driveController.getLeftX(), Constants.DriverConstants.LEFT_X_DEADBAND);
    }

    public static double driverGetLeftY(CommandXboxController driveController) {
        return applyGenericDeadband(driveController.getLeftY(), Constants.DriverConstants.LEFT_Y_DEADBAND);
    }

    public static double driverGetRightX(CommandXboxController driveController) {
        return applyGenericDeadband(driveController.getRightX(), Constants.DriverConstants.RIGHT_X_DEADBAND);
    }

    public static double driverGetRightY(CommandXboxController driveController) {
        return applyGenericDeadband(driveController.getRightY(), Constants.DriverConstants.RIGHT_Y_DEADBAND);
    }

    // Operator Controller Deadband Handling
    public static double operatorGetLeftX(CommandXboxController operatorController) {
        return applyGenericDeadband(operatorController.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND);
    }

    public static double operatorGetLeftY(CommandXboxController operatorController) {
        return applyGenericDeadband(operatorController.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND);
    }

    public static double operatorGetRightX(CommandXboxController operatorController) {
        return applyGenericDeadband(operatorController.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND);
    }

    public static double operatorGetRightY(CommandXboxController operatorController) {
        return applyGenericDeadband(operatorController.getRightY(), Constants.OperatorConstants.RIGHT_Y_DEADBAND);
    }
}
