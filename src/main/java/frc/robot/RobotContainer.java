// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.controller.DriverController;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
    private final SendableChooser<Command> m_autoChooser;

    private final DriverController m_driverController = new DriverController();
    // private final OperatorController m_operatorController = new Controller();

    public static final class subsystems {
        public static final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
        // public static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    }

    public RobotContainer() {
        configureBindings();

        pathplanner();
        m_autoChooser = AutoBuilder.buildAutoChooser();
    }

    private void configureBindings() {
        subsystems.driveSubsystem.setDefaultCommand(
            new FieldOrientedDriveCommand(
                m_driverController::getDriveX,
                m_driverController::getDriveY,
                m_driverController::getDriveRot
            )
        );

        m_driverController.resetGyro().onTrue(
            new InstantCommand(() -> subsystems.driveSubsystem.resetYaw(), new Subsystem[] {subsystems.driveSubsystem})
        );

        m_driverController.getTurnToAngle().whileTrue(
            new TurnToAngleCommand(
                m_driverController::getDriveX,
                m_driverController::getDriveY,
                Rotation2d.fromRadians(0)
            )
        );
    }

    private void pathplanner() {
        subsystems.driveSubsystem.configurePathPlanner();
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
