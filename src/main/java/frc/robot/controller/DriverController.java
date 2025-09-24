package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    public Trigger getTurnToAngle() {
        return a();
    }

    public Trigger resetGyro() {
        return b();
    }
}
