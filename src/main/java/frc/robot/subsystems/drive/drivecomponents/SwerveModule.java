package frc.robot.subsystems.drive.drivecomponents;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

public class SwerveModule {
    private final String m_name;

    private final DriveMotor m_driveMotor;
    private final TurnMotor m_turnMotor;
    
    public SwerveModule(String name, DriveMotor driveMotor, TurnMotor turnMotor) {
        m_name = name;

        m_driveMotor = driveMotor;
        m_turnMotor = turnMotor;
    }

    public void setState(SwerveModuleState moduleState) {
        moduleState.optimize(
            m_turnMotor.getAngle()
        );

        m_turnMotor.setAngle(
            moduleState.angle
        );

        m_driveMotor.setLinearVelocity(
            Units.MetersPerSecond.of(moduleState.speedMetersPerSecond)
        );

        System.out.println("Module Position Angle" + m_turnMotor.getAngle());
        System.out.println("Module Position Linear Velocity" + moduleState.speedMetersPerSecond);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_driveMotor.getDistance(), m_turnMotor.getAngle());
    }
}
