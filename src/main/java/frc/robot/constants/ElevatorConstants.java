package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorConstants {
    public static final class CANIDs {
        public static final int kLeft = 10;
        public static final int kRight = 11;
    }

    public static final class ConversionFactor {
        public static final Distance kElevatorPositionConversion = Units.Meters.of(
            ((1 - kMinRotations) / (kMaxRotations - kMinRotations)) 
                * (kMaxHeight.in(Units.Meters) - kMinHeight.in(Units.Meters)) + kMinHeight.in(Units.Meters)
        );
        
        public static final LinearVelocity kElevatorVelocityConversion = kElevatorPositionConversion.per(Units.Second);
    }

    public static final class PID {
        public static final int kP = 0;
        public static final int kI = 0;
        public static final int kD = 0;
        public static final int kFF = 0;
    }

    public static final double kMinRotations = 0;
    public static final double kMaxRotations = 0;

    public static final Distance kMinHeight = Units.Inches.of(0);
    public static final Distance kMaxHeight = Units.Inches.of(0);

    public static final Current kCurrentLimit = Units.Amps.of(0);
}