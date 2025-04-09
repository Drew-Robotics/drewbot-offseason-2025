package frc.robot.subsystems.drive;

import frc.robot.constants.DriveConstants.CANIDs.DriveMotorCANIDs;
import frc.robot.constants.DriveConstants.CANIDs.TurnMotorCANIDs;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drive.driveOutput.DriveMotor;
import frc.robot.subsystems.drive.driveOutput.Motor;
import frc.robot.subsystems.drive.driveOutput.Motors;
import frc.robot.subsystems.drive.driveOutput.TurnMotor;

public class DriveSubsystem extends Subsystem {
    private final Motors m_motors;

    public DriveSubsystem() {
        super();

        m_motors = new Motors(
            new Motor(
                "Front Left Motor",
                new DriveMotor("Front Left Drive", DriveMotorCANIDs.frontLeft),
                new TurnMotor("Front Left Turn", TurnMotorCANIDs.frontLeft)
            ), 
            new Motor(
                "Front Right Motor",
                new DriveMotor("Front Right Drive", DriveMotorCANIDs.frontRight),
                new TurnMotor("Front Right Turn", TurnMotorCANIDs.frontRight)
            ),
            
            new Motor(
                "Back Left Motor",
                new DriveMotor("Back Left Drive", DriveMotorCANIDs.backLeft), 
                new TurnMotor("Back Left Turn", TurnMotorCANIDs.backLeft)
            ),
            new Motor(
                "Back Right Motor",
                new DriveMotor("Back Right Drive", DriveMotorCANIDs.backRight), 
                new TurnMotor("Back Right Turn", TurnMotorCANIDs.backRight)
            )
        );
    }

    // OVERRIDES

    protected void publishInit() {}

    protected void publishPeriodic() {}
}
