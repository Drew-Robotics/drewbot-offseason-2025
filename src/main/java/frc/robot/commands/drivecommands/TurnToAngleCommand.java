package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer.subsystems;

public class TurnToAngleCommand extends DriveCommand<Double, Double, Rotation2d> {
    public TurnToAngleCommand(Supplier<Double> xVelScalarSup, Supplier<Double> yVelScalarSup, Supplier<Rotation2d> setAngleSup) {
        super(xVelScalarSup, yVelScalarSup, setAngleSup);
    }

    protected void driveFunction(Double xVelScalar, Double yVelScalar, Rotation2d setAngle) {
        subsystems.driveSubsystem.turnToAngleDrive(
            DriveCommand.mkXVel(xVelScalar),
            DriveCommand.mkYVel(yVelScalar),
            setAngle
        );
    }

    /* TODO: IMPLEMENT
    public static alignToAprilTag() {

    } 
    */
}