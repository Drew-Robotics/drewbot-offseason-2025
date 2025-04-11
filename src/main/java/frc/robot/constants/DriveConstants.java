package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

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
        public static final Distance wheelRadius = Units.Meters.of(1);
    }

    public static final class SwerveConstants {

    }
}
