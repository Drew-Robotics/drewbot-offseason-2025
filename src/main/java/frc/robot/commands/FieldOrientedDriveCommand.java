// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.DriveConstants;


public class FieldOrientedDriveCommand extends Command {
    private final DoubleSupplier m_xVel;
    private final DoubleSupplier m_yVel;
    private final DoubleSupplier m_rotVel;

    public FieldOrientedDriveCommand(DoubleSupplier xVel, DoubleSupplier yVel, DoubleSupplier rotVel) {
        m_xVel = xVel;
        m_yVel = yVel;
        m_rotVel = rotVel;

        addRequirements(subsystems.driveSubsystem);
    }

    private void fieldOrientedDrive() {
        double driveScalar = DriveConstants.kDriveScalar.in(Units.MetersPerSecond);

        double xVel = m_xVel.getAsDouble() * driveScalar;
        double yVel = m_yVel.getAsDouble() * driveScalar;
        double rotVel = 
            m_rotVel.getAsDouble() * Math.PI * 2 * DriveConstants.kDriveRotationScalar; // full rotation * scalar

        subsystems.driveSubsystem.fieldOrientedDrive(xVel, yVel, rotVel);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        fieldOrientedDrive();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }
}
