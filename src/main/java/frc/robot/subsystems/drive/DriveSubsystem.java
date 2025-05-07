package frc.robot.subsystems.drive;

import java.util.Arrays;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DriveConstants.kCANIDs.kDriveMotorCANIDs;
import frc.robot.constants.DriveConstants.kCANIDs.kTurnMotorCANIDs;
import frc.robot.constants.DriveConstants.kModuleOffsets;
import frc.robot.constants.DriveConstants.kWheelConstants.kWheelOffsetConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drive.drivecomponents.DriveMotor;
import frc.robot.subsystems.drive.drivecomponents.Gyroscope;
import frc.robot.subsystems.drive.drivecomponents.SwerveModule;
import frc.robot.subsystems.drive.drivecomponents.TurnMotor;

public class DriveSubsystem extends Subsystem {
    private final SwerveModule[] m_swerveModules;

    private final Gyroscope m_gyroscrope;

    private final SwerveDriveKinematics m_kinematics;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private static DriveSubsystem m_instance;
    public static DriveSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new DriveSubsystem();
        
        return m_instance;
    }

    public DriveSubsystem() {
        super();

        m_swerveModules = new SwerveModule[] {
            new SwerveModule(
                "Front Left Motor",
                new DriveMotor("Front Left Drive", kDriveMotorCANIDs.kFrontLeft),
                new TurnMotor("Front Left Turn", kTurnMotorCANIDs.kFrontLeft),
                kModuleOffsets.kFrontLeft
            ), 
            new SwerveModule(
                "Front Right Motor",
                new DriveMotor("Front Right Drive", kDriveMotorCANIDs.kFrontRight),
                new TurnMotor("Front Right Turn", kTurnMotorCANIDs.kFrontRight),
                kModuleOffsets.kFrontRight
            ),
            
            new SwerveModule(
                "Back Left Motor",
                new DriveMotor("Back Left Drive", kDriveMotorCANIDs.kBackLeft), 
                new TurnMotor("Back Left Turn", kTurnMotorCANIDs.kBackLeft),
                kModuleOffsets.kBackLeft
            ),
            new SwerveModule(
                "Back Right Motor",
                new DriveMotor("Back Right Drive", kDriveMotorCANIDs.kBackRight), 
                new TurnMotor("Back Right Turn", kTurnMotorCANIDs.kBackRight),
                kModuleOffsets.kBackRight
            )
        };

        m_gyroscrope = new Gyroscope();

        m_kinematics = new SwerveDriveKinematics(
            kWheelOffsetConstants.kFrontLeftOffset,
            kWheelOffsetConstants.kFrontRightOffset,
            kWheelOffsetConstants.kBackLeftOffset,
            kWheelOffsetConstants.kBackRightOffset
        );

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics, 
            m_gyroscrope.getRotation2dYaw(), 
            getModulePositions(),
            new Pose2d()
        );
    }

    // PRIVATE METHODS

    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(m_swerveModules).map(module -> module.getModulePosition()).toArray(SwerveModulePosition[]::new);
    }

    private void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStates(
            m_kinematics.toSwerveModuleStates(chassisSpeeds)
        );
    }

    private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < 4; i++)
            m_swerveModules[i].setState(swerveModuleStates[i]);
    }

    // PUBLIC METHODS

    /**
     * 
     * @param x x velocity
     * @param y y velocity
     * @param angle rotation velocity, radians per second
     */
    public void fieldOrientedDrive(double x, double y, double angle) {
        setChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x, y, angle, 
                m_gyroscrope.getRotation2dYaw()
            )
        );
    }

    public void resetYaw() {
        m_gyroscrope.resetYaw();
    }

    // OVERRIDES

    @Override
    public void periodic() {
        m_poseEstimator.update(m_gyroscrope.getRotation2d(), getModulePositions());
    }

    // Logging

    protected void publishInit() {}

    protected void publishPeriodic() {}
}
