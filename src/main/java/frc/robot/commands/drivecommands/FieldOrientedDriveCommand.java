// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.RobotContainer.subsystems;

public class FieldOrientedDriveCommand extends DriveCommand<Double, Double, Double> {
    public FieldOrientedDriveCommand(Supplier<Double> xVelScalarSup, Supplier<Double> yVelScalarSup, Supplier<Double> rotVelScalarSup) {
        super(xVelScalarSup, yVelScalarSup, rotVelScalarSup);
    }

    protected void driveFunction(Double xVelScalar, Double yVelScalar, Double rotVelScalar) {
        subsystems.driveSubsystem.fieldOrientedDrive(
            DriveCommand.mkXVel(xVelScalar), 
            DriveCommand.mkYVel(yVelScalar),
            DriveCommand.mkRotVel(rotVelScalar)
        );
    }
}
