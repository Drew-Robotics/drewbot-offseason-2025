package frc.robot.subsystems.drive.drivecomponents;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.kConversionFactors.DriveMotorConversions;
import frc.robot.constants.DriveConstants.kPID.DriveMotorPID;

public class DriveMotor {
    public final int m_CANID;
    public final String m_name;

    public final SparkFlex m_motorController;
    public final RelativeEncoder m_encoder;
    public final SparkClosedLoopController m_closedLoop;

    public DriveMotor(String name, int CANID) {

        m_motorController = new SparkFlex(CANID, MotorType.kBrushless);
        m_CANID = CANID;
        m_name = name;

        m_encoder = m_motorController.getEncoder();
        m_closedLoop = m_motorController.getClosedLoopController();

        SparkFlexConfig configuration = new SparkFlexConfig();

        configuration
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) DriveConstants.kCurrentLimits.kDriveMotorCurrentLimit.in(Units.Amps));
        configuration.encoder
            .positionConversionFactor(DriveMotorConversions.kPositionConversionFactor)
            .velocityConversionFactor(DriveMotorConversions.kVelocityConversionFactor);
        configuration.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                DriveMotorPID.kP, 
                DriveMotorPID.kI, 
                DriveMotorPID.kD
            )
            .outputRange(-1, 1)
            .velocityFF(DriveConstants.kDrivingVelocityFeedForward);
        
        m_motorController.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // SETTERS

    public void setLinearVelocity(LinearVelocity linearVel) {
        SmartDashboard.putNumber(m_name + ": Linear Velocity", linearVel.in(Units.MetersPerSecond));

        setAngularVelocity(
            Units.RadiansPerSecond.of(
                linearVel.in(Units.MetersPerSecond) * 
                    (1 / DriveConstants.kWheelConstants.kWheelRadius.in(Units.Meters))
            )
        );
    }

    public void setAngularVelocity(AngularVelocity angularVel) {
        SmartDashboard.putNumber(m_name + ": Angular Velocity", angularVel.in(RadiansPerSecond));

        m_closedLoop.setReference(
            angularVel.in(Units.RadiansPerSecond), ControlType.kVelocity
        );
    }

    // GETTERS

    public LinearVelocity getLinearVelocity() {
        return Units.MetersPerSecond.of(
            m_encoder.getVelocity() * DriveConstants.kWheelConstants.kWheelRadius.in(Units.Meters)
        );
    }

    public AngularVelocity getAngularVelocity() {
        return Units.RadiansPerSecond.of(m_encoder.getVelocity());
    }

    public Distance getDistance() {
        return Units.Meters.of(m_encoder.getPosition());
    }
}
