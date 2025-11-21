package frc.robot.subsystems.drive.drivecomponents;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.kDriveConversionFactors.TurnMotorConversions;
import frc.robot.constants.DriveConstants.kDrivePID.TurnMotorPID;

public class TurnMotor {
    @SuppressWarnings("unused")
    private final int m_CANID;
    @SuppressWarnings("unused")
    private final String m_name;

    private final SparkMax m_motorController;
    private final AbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_closedLoop;

    public TurnMotor(String name, int CANID) {
        
        m_motorController = new SparkMax(CANID, MotorType.kBrushless);

        m_CANID = CANID;
        m_name = name;

        m_encoder = m_motorController.getAbsoluteEncoder();
        m_closedLoop = m_motorController.getClosedLoopController();

        SparkMaxConfig configuration = new SparkMaxConfig();
  
        configuration
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit((int) DriveConstants.kDriveCurrentLimits.kTurnMotorCurrentLimit.in(Units.Amps));
        configuration.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(TurnMotorConversions.kPositionConversionFactor.in(Units.Radians))
            .velocityConversionFactor(TurnMotorConversions.kVelocityConversionFactor.in(Units.RadiansPerSecond));
        configuration.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(
                TurnMotorPID.kP, 
                TurnMotorPID.kI, 
                TurnMotorPID.kD
            )
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(
                0, 
                TurnMotorConversions.kPositionConversionFactor.in(Units.Radians)
            );
        
        m_motorController.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAngle(Rotation2d angle) {
        m_closedLoop.setReference(angle.getRadians(), ControlType.kPosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_encoder.getPosition());
    }
}

