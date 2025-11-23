package frc.robot.subsystems.drive.drivecomponents;

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
            .positionConversionFactor(DriveMotorConversions.kPositionConversionFactor.in(Units.Meters))
            .velocityConversionFactor(DriveMotorConversions.kVelocityConversionFactor.in(Units.MetersPerSecond));
        configuration.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                DriveMotorPID.kP, 
                DriveMotorPID.kI, 
                DriveMotorPID.kD,
                DriveMotorPID.kFF
            )
            .outputRange(-1, 1);
        
        m_motorController.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // SETTERS

    public void setLinearVelocity(LinearVelocity linearVel) {
        SmartDashboard.putNumber(m_name + ": Set Linear Velocity m/s", linearVel.in(Units.MetersPerSecond));

        m_closedLoop.setReference(linearVel.in(Units.MetersPerSecond), ControlType.kVelocity);
    }

    // GETTERS

    public LinearVelocity getLinearVelocity() {
        return Units.MetersPerSecond.of(m_encoder.getVelocity());
    }

    public Distance getDistance() {
        SmartDashboard.putNumber(m_name + ": Distance m", m_encoder.getPosition());

        return Units.Meters.of(m_encoder.getPosition());
    }
}
