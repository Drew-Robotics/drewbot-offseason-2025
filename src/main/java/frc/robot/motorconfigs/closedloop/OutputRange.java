package frc.robot.motorconfigs.closedloop;

public class OutputRange {
    private final double m_min;
    private final double m_max;
    
    public OutputRange(double min, double max) {
        m_min = min;
        m_max = max;
    }

    public double getMin() { return m_min; }
    public double getMax() { return m_max; }
}
