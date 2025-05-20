package frc.robot.constants;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Units;

public class CoralConstants {
    public static final double kP = 12; // 1.5
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.7;

    public static final AngularVelocity kMaxVel = Units.DegreesPerSecond.of(300);
    public static final AngularAcceleration kMaxAccel = Units.DegreesPerSecondPerSecond.of(1000);


    public static final int kCoralArm = 20;
    public static final int kCoralArmEncoder = 22;
}