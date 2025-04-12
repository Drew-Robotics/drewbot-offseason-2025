package frc.robot.constants;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
    public static final class CANIDs {
        public static final class DriveMotorCANIDs {
            public static final int frontLeft = 0;
            public static final int frontRight = 0;

            public static final int backLeft = 0;
            public static final int backRight = 0;
        }

        public static final class TurnMotorCANIDs {
            public static final int frontLeft = 0;
            public static final int frontRight = 0;
            
            public static final int backLeft = 0;
            public static final int backRight = 0;
        }
    }

    public static final class WheelConstants {
        public static final Distance kWheelRadius = Units.Meters.of(1);

        public static final class WheelOffsetConstants {
            public static final Translation2d kFrontLeftOffset = 
                new Translation2d(0, 0);
            public static final Translation2d kFrontRightOffset =
                new Translation2d(0, 0);
            public static final Translation2d kBackLeftOffset = 
                new Translation2d(0, 0);
            public static final Translation2d kBackRightOffset =
                new Translation2d(0, 0);
        }
    }

    public static final NavXComType gyroComType = NavXComType.kUSB1;

    public static final LinearVelocity driveScalar = Units.MetersPerSecond.of(1);
    public static final double driveRotationScalar = 0.5;
}
