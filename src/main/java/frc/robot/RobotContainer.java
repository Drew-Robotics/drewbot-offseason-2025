// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.controller.DriverController;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    private final DriverController m_driverController = new DriverController();
    // private final OperatorController m_operatorController = new Controller();

    public static final class subsystems {
        public static final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
        // public static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    }

    private void configureBindings() {
        subsystems.driveSubsystem.setDefaultCommand(
            new FieldOrientedDriveCommand(
                m_driverController::getDriveX,
                m_driverController::getDriveY,
                m_driverController::getDriveRot
            )
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
