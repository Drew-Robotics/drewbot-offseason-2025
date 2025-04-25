package frc.robot.subsystems.drive.drivecomponents;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DriveConstants;
import frc.robot.motorconfigs.MotorConfigTools;
import frc.robot.motorconfigs.closedloop.OutputRange;

public class TurnMotor extends SparkMax {
    public final int m_CANID;
    public final String m_name;

    public final AbsoluteEncoder m_encoder;
    public final SparkClosedLoopController m_closedLoop;

    public TurnMotor(String name, int CANID) {
        super(CANID, MotorType.kBrushless);

        m_CANID = CANID;
        m_name = name;

        m_encoder = this.getAbsoluteEncoder();
        m_closedLoop = this.getClosedLoopController();

        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration
            .apply(
                MotorConfigTools.mkEncoderConfig(DriveConstants.kConversionFactors.TurnMotorConversions.kConversionFactors)
            )
            .apply(
                MotorConfigTools.mkMotorConfig(IdleMode.kBrake, DriveConstants.kCurrentLimits.kTurnMotorCurrentLimit)
                    .inverted(true)
            )
            .apply(
                MotorConfigTools.mkClosedLoopConfig(
                    FeedbackSensor.kAbsoluteEncoder, 
                    DriveConstants.kPID.TurnMotorPID.kPID,
                    new OutputRange(-1, 1)
                )
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(
                    0, 
                    DriveConstants.kConversionFactors.TurnMotorConversions.kPositionConversionFactor
                )
            );
        
        MotorConfigTools.configureMotor(configuration, this);
    }

    public void setAngle(Rotation2d angle) {
        m_closedLoop.setReference(angle.getRadians(), ControlType.kPosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_encoder.getPosition());
    }
}

