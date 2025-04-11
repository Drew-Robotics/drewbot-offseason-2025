package frc.robot.subsystems.drive.driveOutput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

public class Motor {
    private final String m_name;

    private final DriveMotor m_driveMotor;
    private final TurnMotor m_turnMotor;
    
    public Motor(String name, DriveMotor driveMotor, TurnMotor turnMotor) {
        m_name = name;

        m_driveMotor = driveMotor;
        m_turnMotor = turnMotor;
    }

    public void setState(SwerveModuleState moduleState) {
        moduleState.optimize(
            new Rotation2d(m_turnMotor.getAngle())
        );

        m_turnMotor.setAngle(
            moduleState.angle.getMeasure()
        );

        m_driveMotor.setLinearVelocity(
            Units.MetersPerSecond.of(moduleState.speedMetersPerSecond)
        );
    }
}
