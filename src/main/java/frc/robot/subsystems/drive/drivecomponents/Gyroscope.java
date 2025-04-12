package frc.robot.subsystems.drive.drivecomponents;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DriveConstants;

public class Gyroscope extends AHRS {
    public Gyroscope() {
        super(DriveConstants.gyroComType);
    }

    public Rotation2d getRotation2dYaw() {
        return Rotation2d.fromDegrees(this.getYaw());
    }

    public void resetYaw() {
        this.zeroYaw();
    }
}
