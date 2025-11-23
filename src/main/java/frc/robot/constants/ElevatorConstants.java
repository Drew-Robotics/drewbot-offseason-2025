package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorConstants {
    public static final class CANIDs {
        public static final int kLeft = 10;
        public static final int kRight = 11;
    }

    public static final class ConversionFactor {
        //public static final LinearVelocity kElevatorVelocityConversion = kElevatorPositionConversion.per(Units.Second);
    }

    public static final class PID {
        public static final int kP = 0;
        public static final int kI = 0;
        public static final int kD = 0;
        public static final int kFF = 0;
    }

    public static final Angle kMinRotations = Units.Rotation.of(0);
    public static final Angle kMaxRotations = Units.Rotation.of(0);

    public static final Distance kMinHeight = Units.Inches.of(0);
    public static final Distance kMaxHeight = Units.Inches.of(0);

    public static final Current kCurrentLimit = Units.Amps.of(0);
}