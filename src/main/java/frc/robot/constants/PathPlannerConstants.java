package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.units.Units;
import frc.robot.constants.MeasureConstants.kSwerveMeasures;

public class PathPlannerConstants {

    public static final ModuleConfig kModuleConfig = new ModuleConfig(
        MeasureConstants.kSwerveMeasures.kWheelRadius, 
        Units.MetersPerSecond.of(5.65),
        MeasureConstants.kSwerveMeasures.kCoefficientOfFriction, 
        DriveConstants.kSwerveCalculations.kDriveMotor, 
        kSwerveMeasures.kDrivingMotorCurrentLimit, 
        kSwerveMeasures.kNumMotors
    );

    public static final RobotConfig kRobotConfig = new RobotConfig(
        Units.Pounds.of(110),
        Units.KilogramSquareMeters.of(4),
        kModuleConfig,
        DriveConstants.kKinematics.getModules()
    );

    public final static class kPID {
        public final static class kTranslationPID {
            public final static double kP = 0.1;
            public final static double kI = 0;
            public final static double kD = 0;

            public static final PIDConstants pidConstants = new PIDConstants(
                kP, kI, kD
            );
        }

        public final static class kRotationPID {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final PIDConstants pidConstants = new PIDConstants(
                kP, kI, kD
            );
        }
    }

    public final static PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
        kPID.kTranslationPID.pidConstants, kPID.kRotationPID.pidConstants
    );
}
