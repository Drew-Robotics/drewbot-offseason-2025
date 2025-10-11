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
                // Sets "idle" mode for motor, i.e. here it stops the motor from moving when its not doing anything
            .smartCurrentLimit((int) DriveConstants.kCurrentLimits.kDriveMotorCurrentLimit.in(Units.Amps));
                // Just a current limit for the motor to make sure we dont explode the motor
        configuration.encoder
            .positionConversionFactor(DriveMotorConversions.kPositionConversionFactor.in(Units.Meters))
                // sets the position conversion factor, 
                // you don't need to check that its just conversions from gear ratios and radians to linear stuff 
            .velocityConversionFactor(DriveMotorConversions.kVelocityConversionFactor.in(Units.MetersPerSecond));
                // same
        configuration.closedLoop
            // PID
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Sets the encoder for this, depending on what kind of encoder it uses is important
                // sometimes there will be an absolute encoder so change accordingly
            .pidf(
                DriveMotorPID.kP, 
                DriveMotorPID.kI, 
                DriveMotorPID.kD,
                DriveMotorPID.kFF
            )
                // pid constants
            .outputRange(-1, 1);
                // usu. -1 to 1, can be contrained more tightly
                // unless you know what you are doing dont go less than -1 or greater than 1
        
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
        return Units.Meters.of(m_encoder.getPosition());
    }
}
