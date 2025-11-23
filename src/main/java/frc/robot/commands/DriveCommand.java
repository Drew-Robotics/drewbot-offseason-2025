package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.DriveConstants;

/** 
 * A command that depends on the driveSubsystem and manages an X value, Y value, and a Rotation value
 */
public abstract class DriveCommand<X, Y, R> extends Command {
    protected final Supplier<X> m_x;
    protected final Supplier<Y> m_y;
    protected final Supplier<R> m_rot;

    public DriveCommand(Supplier<X> x, Supplier<Y> y, Supplier<R> rot) {
        m_x = x;
        m_y = y;
        m_rot = rot;

        addRequirements(subsystems.driveSubsystem);
    }

    protected abstract void driveFunction(X x, Y y, R rot);

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveFunction(m_x.get(), m_y.get(), m_rot.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }

    /* HELPER FUNCTIONS */

    public static LinearVelocity mkXVel(Double xVelScalar) {
        return DriveConstants.kMaxVels.kMaxDrive.times(
            (DriveConstants.kXInverted ? -1.0 : 1.0) * xVelScalar
        );
    }

    public static LinearVelocity mkYVel(Double yVelScalar) {
        return DriveConstants.kMaxVels.kMaxDrive.times(
            (DriveConstants.kYInverted ? -1.0 : 1.0) * yVelScalar
        );
    }

    public static AngularVelocity mkRotVel(Double rotVelScalarSup) {
        return DriveConstants.kMaxVels.kMaxAngular.times(rotVelScalarSup);
    }
}
