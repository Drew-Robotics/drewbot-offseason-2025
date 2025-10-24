package frc.robot.subsystems.drive.drivecomponents;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DriveConstants.kGyro;

public class Gyroscope {
    private final AHRS m_gyro;

    private Rotation2d m_offset = Rotation2d.kZero;

    public Gyroscope() {
        m_gyro = new AHRS(NavXComType.kMXP_SPI);
    }

    private Rotation2d getYawRaw() {
        return Rotation2d.fromDegrees(m_gyro.getYaw());
    }
   
    private Rotation2d normalizeAngle(Rotation2d angle) {
        return Rotation2d.fromDegrees(angle.getDegrees() % 360);
    }

    // PUBLIC FUNCTIONS

    public void resetYaw() {
        m_offset = getYawRaw();
    }

    public Rotation2d getYaw() {
        return normalizeAngle(
            getYawRaw()
                .minus(kGyro.kGyroOffset)
                .minus(m_offset)
        ).times(kGyro.kGyroInverted ? -1 : 1);
    }
}
