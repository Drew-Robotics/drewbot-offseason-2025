package frc.robot.controller;

import frc.robot.constants.ControllerConstants;

public class DriverController extends Controller {
    public DriverController() {
        super(ControllerConstants.kDriverPort);
    }

    public double getDriveX() {
        return getLeftX();
    }

    public double getDriveY() {
        return getLeftY();
    }

    public double getDriveRot() {
        return getRightX();
    }
}
