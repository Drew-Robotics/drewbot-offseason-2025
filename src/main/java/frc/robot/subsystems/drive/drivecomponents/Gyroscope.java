package frc.robot.subsystems.drive.drivecomponents;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.kGyro;

public class Gyroscope {
    private final AHRS m_gyro;

    public Gyroscope() {
        m_gyro = new AHRS(kGyro.kGyroComType);
    }

    public Rotation2d getYaw() {
        Angle angleMeasure = Units.Degrees.of(
            m_gyro.getYaw() * (DriveConstants.kGyro.kGyroInverted ? -1.0 : 1.0)
        );
        return new Rotation2d(angleMeasure);    
    }

    public void resetYaw() {
        m_gyro.zeroYaw();
    }
}
