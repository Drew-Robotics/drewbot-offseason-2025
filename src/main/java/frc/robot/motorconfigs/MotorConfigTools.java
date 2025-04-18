package frc.robot.motorconfigs;

import java.util.stream.Stream;

import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import frc.robot.motorconfigs.closedloop.OutputRange;
import frc.robot.motorconfigs.closedloop.PID;
import frc.robot.motorconfigs.encoder.ConversionFactors;

public class MotorConfigTools {

    // MOTOR

    public static SparkBaseConfig mkMotorConfig(IdleMode idleMode, Current currentLimit) {
        return new SparkMaxConfig().smartCurrentLimit((int) currentLimit.in(Units.Amps)).idleMode(idleMode);
    }

    // ENCODER

    public static EncoderConfig mkEncoderConversions(ConversionFactors conversions) {
        return 
            new EncoderConfig()
                .positionConversionFactor(conversions.getPositionConversionFactor())
                .velocityConversionFactor(conversions.getVelocityConversionFactor());
    }

    public static AbsoluteEncoderConfig mkAbsoluteEncoderConversions(ConversionFactors conversions) {
        return
            new AbsoluteEncoderConfig()
                .positionConversionFactor(conversions.getPositionConversionFactor())
                .velocityConversionFactor(conversions.getVelocityConversionFactor());
    }

    public static EncoderConfig mkEncoderConfig(ConversionFactors conversions) {
        return mkEncoderConfig(conversions);
    }

    public static EncoderConfig mkAbsoluteEncoderConfig(ConversionFactors conversions) {
        return mkAbsoluteEncoderConfig(conversions);
    }

    // CLOSED LOOP

    public static ClosedLoopConfig mkClosedLoopConfig(FeedbackSensor feedbackSensor, PID pid, OutputRange outputRange) {
        return
            new ClosedLoopConfig()
                .feedbackSensor(feedbackSensor)
                .pid(pid.getP(), pid.getI(), pid.getD())
                .outputRange(outputRange.getMin(), outputRange.getMax());
    }
}
