package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class MeasureConstants {
    public static final class kSwerveMeasures {
        public static final Distance kWheelDiameter = Units.Inches.of(2.82);
        public static final Distance kWheelCircumference = kWheelDiameter.times(Math.PI);

        public static final double kDrivingMotorReduction = 5.5d;

        public static final double kCoefficientOfFriction = 1.45d;

        public static final int kNumMotors = 4;

        public static final Current kDrivingMotorCurrentLimit = Amps.of(40);
    }

    public static final class kBodyMeasures {
        public static final Distance kWheelBase = Units.Inches.of(23);
        public static final Distance kTrackWidth = Units.Inches.of(23);  
    }
}
