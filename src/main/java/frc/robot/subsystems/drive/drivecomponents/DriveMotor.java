package frc.robot.subsystems.drive.drivecomponents;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.DriveConstants;
import frc.robot.motorconfigs.MotorConfigTools;
import frc.robot.motorconfigs.closedloop.OutputRange;

public class DriveMotor extends SparkFlex {
    public final int m_CANID;
    public final String m_name;

    public final RelativeEncoder m_encoder;
    public final SparkClosedLoopController m_closedLoop;

    public DriveMotor(String name, int CANID) {
        super(CANID, MotorType.kBrushless);

        m_CANID = CANID;
        m_name = name;

        m_encoder = this.getEncoder();
        m_closedLoop = this.getClosedLoopController();

        SparkFlexConfig configuration = new SparkFlexConfig();

        configuration
            .apply(MotorConfigTools.mkMotorConfig(IdleMode.kBrake, DriveConstants.kCurrentLimits.kDriveMotorCurrentLimit))
            .apply(
                MotorConfigTools.mkClosedLoopConfig(
                    FeedbackSensor.kPrimaryEncoder, 
                    DriveConstants.kPID.DriveMotorPID.kPID, 
                    new OutputRange(-1, 1)
                ).velocityFF(DriveConstants.kDrivingVelocityFeedForward)
            )
            .apply(
                MotorConfigTools.mkEncoderConfig(DriveConstants.kConversionFactors.DriveMotorConversions.kConversionFactors)
                    .inverted(true)
            );
        
        MotorConfigTools.configureMotor(configuration, this);
    }

    // SETTERS

    public void setLinearVelocity(LinearVelocity linearVel) {
        setAngularVelocity(
            Units.RadiansPerSecond.of(
                linearVel.in(Units.MetersPerSecond) * 
                    (1 / DriveConstants.kWheelConstants.kWheelRadius.in(Units.Meters))
            )
        );
    }

    public void setAngularVelocity(AngularVelocity angularVel) {
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
