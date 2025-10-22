package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.DriveConstants;

public class TurnToAngleCommand extends Command {
    private final Rotation2d m_setAngle;

    private final DoubleSupplier m_xVel;
    private final DoubleSupplier m_yVel;


    public TurnToAngleCommand(DoubleSupplier xVel, DoubleSupplier yVel, Rotation2d rot) {
        m_setAngle = rot;

        m_xVel = xVel;
        m_yVel = yVel;

        addRequirements(subsystems.driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        LinearVelocity xVel =  
            DriveConstants.kMaxVels.kMaxDrive.times(
                (DriveConstants.kXInverted ? -1.0 : 1.0) * m_xVel.getAsDouble()
            );
        LinearVelocity yVel = 
            DriveConstants.kMaxVels.kMaxDrive.times(
                (DriveConstants.kYInverted ? -1.0 : 1.0) * m_yVel.getAsDouble()
            );

        subsystems.driveSubsystem.turnToAngleDrive(
            xVel,
            yVel,
            m_setAngle
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}