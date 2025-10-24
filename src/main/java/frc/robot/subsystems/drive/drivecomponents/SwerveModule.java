package frc.robot.subsystems.drive.drivecomponents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

public class SwerveModule {
    private final String m_name;

    private final DriveMotor m_driveMotor;
    private final TurnMotor m_turnMotor;

    private final Rotation2d m_angularOffset;
    
    public SwerveModule(String name, DriveMotor driveMotor, TurnMotor turnMotor, Rotation2d angularOffset) {
        m_name = name;

        m_driveMotor = driveMotor;
        m_turnMotor = turnMotor;

        m_angularOffset = angularOffset;
    }

    public void setState(SwerveModuleState robotRelativeState) {
        setRobotRelativeState(
            toModuleRelativeState(robotRelativeState)
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_driveMotor.getDistance(), m_turnMotor.getAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(m_driveMotor.getLinearVelocity(), m_turnMotor.getAngle());
    }

    private SwerveModuleState toModuleRelativeState(SwerveModuleState moduleState) {
        return new SwerveModuleState(
            moduleState.speedMetersPerSecond,
            moduleState.angle.plus(m_angularOffset)
        );
    }

    private void setRobotRelativeState(SwerveModuleState moduleState) {
        moduleState.optimize(m_turnMotor.getAngle());

        m_turnMotor.setAngle(moduleState.angle);

        m_driveMotor.setLinearVelocity(
            Units.MetersPerSecond.of(moduleState.speedMetersPerSecond)
        );
    }
}