package frc.robot.commands;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.kPID.RotationPID;

public class TurnToAngleCommand extends Command {
    private final Rotation2d m_commandedAngle;
    private final ProfiledPIDController m_pidController;

    private final DoubleSupplier m_xVelocity;
    private final DoubleSupplier m_yVelocity;


    public TurnToAngleCommand(DoubleSupplier xVel, DoubleSupplier yVel, Rotation2d rot) {
        m_commandedAngle = rot;

        m_xVelocity = xVel;
        m_yVelocity = yVel;

        m_pidController = new ProfiledPIDController(
            RotationPID.kP,
            RotationPID.kI,
            RotationPID.kD,
            new Constraints(
                RotationPID.Constraints.maxVelocity.in(RadiansPerSecond), 
                RotationPID.Constraints.maxAcceleration.in(RadiansPerSecondPerSecond)
            )
        );

        m_pidController.enableContinuousInput(Math.PI * -1, Math.PI);

        addRequirements(subsystems.driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Rotation2d rotation = subsystems.driveSubsystem.getPose().getRotation();
        double commanded = m_pidController.calculate(rotation.getRadians(), m_commandedAngle.getRadians() - Math.PI);

        subsystems.driveSubsystem.fieldOrientedDrive(
            m_xVelocity.getAsDouble() * (DriveConstants.kXInverted ? -1.0 : 1.0) * DriveConstants.kMaxDriveVel,
            m_yVelocity.getAsDouble() * (DriveConstants.kYInverted ? -1.0 : 1.0) * DriveConstants.kMaxDriveVel,
            commanded * -1
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}