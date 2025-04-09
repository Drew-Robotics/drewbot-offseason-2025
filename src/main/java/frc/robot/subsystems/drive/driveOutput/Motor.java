package frc.robot.subsystems.drive.driveOutput;

public class Motor {
    private final String m_name;

    private final DriveMotor m_driveMotorController;
    private final TurnMotor m_turningMotorController;
    
    public Motor(String name, DriveMotor driveMotor, TurnMotor turnMotor) {
        m_name = name;

        m_driveMotorController = driveMotor;
        m_turningMotorController = turnMotor;
    }
}
