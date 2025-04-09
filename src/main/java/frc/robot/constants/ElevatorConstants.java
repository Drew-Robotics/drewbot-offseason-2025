package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.motorconfigs.closedloop.OutputRange;
import frc.robot.motorconfigs.closedloop.PIDF;
import frc.robot.motorconfigs.encoder.ConversionFactors;

public class ElevatorConstants {
    public static final class CANIDs {
        public static final int kLeft = 0;
        public static final int kRight = 0;
    }

    public static final class ConversionFactor {
        public static final Distance kElevatorPositionConversion = Units.Meters.of(
            ((1 - kMinRotations) / (kMaxRotations - kMinRotations)) 
                * (kMaxHeight.in(Units.Meters) - kMinHeight.in(Units.Meters)) + kMinHeight.in(Units.Meters)
        );
        public static final LinearVelocity kElevatorVelocityConversion = kElevatorPositionConversion.per(Units.Second);

        public static final ConversionFactors kConversions = new ConversionFactors(
            kElevatorPositionConversion.in(Units.Meters), 
            kElevatorVelocityConversion.in(Units.MetersPerSecond)
        );
    }

    public static final class ClosedLoop {
        public static final int kP = 0;
        public static final int kI = 0;
        public static final int kD = 0;
        public static final int kFF = 0;
        
        public static final PIDF kPID = new PIDF(kP, kI, kD, kFF);

        public static final OutputRange outputRange = new OutputRange(0, 0);
    }

    public static final double kMinRotations = 0;
    public static final double kMaxRotations = 0;

    public static final Distance kMinHeight = Units.Inches.of(0);
    public static final Distance kMaxHeight = Units.Inches.of(0);

    public static final Current kCurrentLimit = Units.Amps.of(0);
}