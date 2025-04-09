package frc.robot.motorconfigs.encoder;

public class ConversionFactors {
    private final double m_positionConversionFactor;
    private final double m_velocityConversionFactor;

    public ConversionFactors(double positionConversion, double veloctiyConversion) {
        m_positionConversionFactor = positionConversion;
        m_velocityConversionFactor = veloctiyConversion;
    }

    public double getPositionConversionFactor() { return m_positionConversionFactor; }
    public double getVelocityConversionFactor() { return m_velocityConversionFactor; }
}