package frc.robot.subsystems.drive.driveOutput;

public class Motors {
    public final Motor m_frontLeft;
    public final Motor m_frontRight;

    public final Motor m_backLeft;
    public final Motor m_backRight;

    public Motors(
        Motor frontLeft, 
        Motor frontRight, 
        Motor backLeft, 
        Motor backRight
    ) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;

        m_backLeft = backLeft;
        m_backRight = backRight;
    }
}
