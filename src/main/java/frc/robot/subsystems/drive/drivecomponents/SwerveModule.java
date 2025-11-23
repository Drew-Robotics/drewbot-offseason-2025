package frc.robot.subsystems.drive.drivecomponents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

public class SwerveModule {
    @SuppressWarnings("unused")
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


    /**
     * @param moduleState robot relative module state
     */
    public void setState(SwerveModuleState moduleState) {
        SwerveModuleState moduleRelativeState = new SwerveModuleState(
            moduleState.speedMetersPerSecond,
            toModuleRelativeAngle(moduleState.angle)
        );

        moduleRelativeState.optimize(m_turnMotor.getAngle());

        m_turnMotor.setAngle(moduleRelativeState.angle);

        m_driveMotor.setLinearVelocity(
            Units.MetersPerSecond.of(moduleRelativeState.speedMetersPerSecond)
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_driveMotor.getDistance(), toRobotRelativeAngle(m_turnMotor.getAngle()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(m_driveMotor.getLinearVelocity(), toRobotRelativeAngle(m_turnMotor.getAngle()));
    }

    /**
     * @param moduleState robot relative module angle
     * @return module relative angle
     */
    private Rotation2d toModuleRelativeAngle(Rotation2d angle) {
        return angle.plus(m_angularOffset);
    }

    /**
     * @param moduleState module relative angle
     * @return robot relative angle
     */
    private Rotation2d toRobotRelativeAngle(Rotation2d angle) {
        return angle.minus(m_angularOffset);
    }
}