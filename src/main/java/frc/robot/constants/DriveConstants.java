package frc.robot.constants;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
    public static final class kCANIDs {
        public static final class kDriveMotorCANIDs {
            public static final int kFrontLeft = 5;
            public static final int kFrontRight = 7;

            public static final int kBackLeft = 3;
            public static final int kBackRight = 1;
        }

        public static final class kTurnMotorCANIDs {
            public static final int kFrontLeft = 6;
            public static final int kFrontRight = 8;

            
            public static final int kBackLeft = 4;
            public static final int kBackRight = 2;
        }
    }

    public static final class kWheelConstants {
        public static final Distance kWheelRadius = Units.Inches.of(3);

        public static final class kWheelOffsetConstants {
            public static final Translation2d kFrontLeftOffset = 
                new Translation2d(kWheelBase.div(2), kTrackWidth.div(2));
            public static final Translation2d kFrontRightOffset =
                new Translation2d(kWheelBase.div(2), kTrackWidth.div(2).negate());
            public static final Translation2d kBackLeftOffset = 
                new Translation2d(kWheelBase.div(2).negate(), kTrackWidth.div(2));
            public static final Translation2d kBackRightOffset =
                new Translation2d(kWheelBase.div(2).negate(), kTrackWidth.div(2).negate());
        }
    }

    public static final class kCurrentLimits {
        public static final Current kDriveMotorCurrentLimit = Units.Amps.of(40);
        public static final Current kTurnMotorCurrentLimit = Units.Amps.of(20);
    }

    public static final class kConversionFactors {
        public static final class DriveMotorConversions {
            public static final int kDrivingMotorPinionTeeth = 14;

            // public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60.0;
            public static final double kWheelCircumferenceMeters = 
                kWheelConstants.kWheelRadius.in(Units.Meters) * Math.PI * 2;

            public static final double kDrivingMotorReduction = (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);
            // public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) 
            //    / kDrivingMotorReduction;

            public static final double kPositionConversionFactor = kDrivingMotorReduction * kWheelCircumferenceMeters;
            public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
        }

        public static final class TurnMotorConversions {
            public static final double kPositionConversionFactor = 2 * Math.PI;
            public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
        }
    }

    public static final class kPID {
        public static final class DriveMotorPID {
            public static final double kP = 0.005;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static final class TurnMotorPID {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static final class RotationPID {
            public static final double kP = 40;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final class Constraints {
                public static final AngularVelocity maxVelocity = Units.DegreesPerSecond.of(180);
                public static final AngularAcceleration maxAcceleration = maxVelocity.times(2).per(Units.Second);
            }
        }
    }

    public static final class kModuleOffsets {
        public static final Rotation2d kFrontLeft = Rotation2d.fromDegrees(-90);
        public static final Rotation2d kFrontRight = Rotation2d.fromDegrees(0);
        public static final Rotation2d kBackLeft = Rotation2d.fromDegrees(180);
        public static final Rotation2d kBackRight = Rotation2d.fromDegrees(90);
    }

    public static final class kGyro {
        public static final boolean kGyroInverted = true;
        public static final Rotation2d kGyroOffset = Rotation2d.fromDegrees(90);
        public static final NavXComType kGyroComType = NavXComType.kUSB1;
    }

    public static final double kFreeSpeedRpm = 5676;
    public static final double kDrivingVelocityFeedForward = 1.0 / kFreeSpeedRpm;

    public static final Distance kWheelBase = Units.Inches.of(23);
    public static final Distance kTrackWidth = Units.Inches.of(23);  

    public static final boolean kXInverted = false;
    public static final boolean kYInverted = true;

    public static final LinearVelocity kMaxDriveVel = Units.MetersPerSecond.of(3).div(2);
    public static final AngularVelocity kMaxAngularVel = Units.RadiansPerSecond.of(2 * Math.PI).div(2);
}