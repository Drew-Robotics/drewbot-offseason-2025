package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.kCANIDs.kDriveMotorCANIDs;
import frc.robot.constants.DriveConstants.kCANIDs.kTurnMotorCANIDs;
import frc.robot.constants.DriveConstants.kModuleOffsets;
import frc.robot.constants.DriveConstants.kSwerveCalculations;
import frc.robot.constants.DriveConstants.kPID.RotationPID;
import frc.robot.constants.PathPlannerConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drive.drivecomponents.DriveMotor;
import frc.robot.subsystems.drive.drivecomponents.Gyroscope;
import frc.robot.subsystems.drive.drivecomponents.SwerveModule;
import frc.robot.subsystems.drive.drivecomponents.TurnMotor;

public class DriveSubsystem extends Subsystem {
    private final SwerveModule[] m_swerveModules;

    private final Gyroscope m_gyro;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final ProfiledPIDController m_rotationPID;

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

        m_gyro = new Gyroscope();

        m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kKinematics, 
            m_gyro.getYaw(), 
            getModulePositions(),
            new Pose2d()
        );

        m_rotationPID = new ProfiledPIDController(
            RotationPID.kP,
            RotationPID.kI,
            RotationPID.kD,
            new Constraints(
                RotationPID.Constraints.maxVelocity.in(Units.RadiansPerSecond), 
                RotationPID.Constraints.maxAcceleration.in(Units.RadiansPerSecondPerSecond)
            )
        );

        m_rotationPID.enableContinuousInput(Math.PI * -1, Math.PI);
    }

    // PRIVATE METHODS

    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(m_swerveModules).map(module -> module.getModulePosition()).toArray(SwerveModulePosition[]::new);
    }

    private SwerveModuleState[] getModuleStates() {
        return Arrays.stream(m_swerveModules).map(module -> module.getModuleState()).toArray(SwerveModuleState[]::new);
    }


    /**
     * Robot Relative ChassisSpeeds
     */
    private void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, kSwerveCalculations.kDriveWheelFreeSpeed);

        setModuleStates(states);
    }

    private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (
            int i = 0; 
            i < Math.min(swerveModuleStates.length, m_swerveModules.length); 
            i++
        ) m_swerveModules[i].setState(swerveModuleStates[i]);
    }

    // PUBLIC METHODS

    /**
     * Field relative drive
     * @param xVel x velocity
     * @param yVel y velocity
     * @param rotVel rotation velocity, radians per second
     */
    public void fieldOrientedDrive(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rotVel) {
        setChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVel.in(Units.MetersPerSecond), 
                yVel.in(Units.MetersPerSecond), 
                rotVel.in(Units.RadiansPerSecond), 
                m_gyro.getYaw()
            )
        );
    }

    /**
     * 
     * @param xVel x velocity
     * @param yVel y velocity
     * @param angle angle to set the robot to
     */
    public void turnToAngleDrive(LinearVelocity xVel, LinearVelocity yVel, Rotation2d angle) {
        AngularVelocity commandedAngleVel = 
            RadiansPerSecond.of(
                m_rotationPID.calculate(
                    getPose().getRotation().getRadians(), 
                    angle.getRadians()
                )
            );

        fieldOrientedDrive(
            xVel,
            yVel,
            commandedAngleVel
        );
    }

    public void resetYaw() {
        m_gyro.resetYaw();
    }

    // PATHPLANNER

    /**
     * Resets the pose of the robot to the selected pose
     */
    private void resetPose(Pose2d pose) {
        m_poseEstimator.resetPose(pose);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * returns robot relative ChassisSpeeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kKinematics.toChassisSpeeds(getModuleStates());  
    }

    public void setupPathplanner() {
        BooleanSupplier isRedSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getChassisSpeeds, 
            (speeds, _ignore) -> setChassisSpeeds(speeds), 
            PathPlannerConstants.kDriveController, 
            PathPlannerConstants.kRobotConfig,
            isRedSupplier, 
            this
        );
    }

    // OVERRIDES

    @Override
    public void periodic() {
        super.periodic();

        m_poseEstimator.update(m_gyro.getYaw(), getModulePositions());
    }

    // LOGGING

    protected void publishInit() {}

    protected void publishPeriodic() {
        SmartDashboard.putNumber("Gyroscope Yaw", m_gyro.getYaw().getRadians());
        SmartDashboard.putNumber("Gyroscope Yaw Degrees", m_gyro.getYaw().getDegrees());
    }
}