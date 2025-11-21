package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class MeasureConstants {
    public static final class kSwerveMeasures {
        public static final Distance kWheelRadius = Units.Inches.of(3);
        public static final Distance kWheelCircumference = kWheelRadius.times(Math.PI * 2);

        public static final int kDrivingMotorPinionTeeth = 12;
        public static final double kDrivingMotorReduction = (45d * 22d) / (kDrivingMotorPinionTeeth * 15d);

        public static final double kCoefficientOfFriction = 1.45;

        public static final int kNumMotors = 4;

        public static final Current kDrivingMotorCurrentLimit = Amps.of(40);
    }

    public static final class kBodyMeasures {
        public static final Distance kWheelBase = Units.Inches.of(23);
        public static final Distance kTrackWidth = Units.Inches.of(23);  
    }
}
