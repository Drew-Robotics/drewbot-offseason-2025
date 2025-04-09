package frc.robot.motorconfigs.closedloop;

public class PID {
    private final double m_P;
    private final double m_I;
    private final double m_D;

    public PID(double P, double I, double D) {
        m_P = P;
        m_I = I;
        m_D = D;
    }

    public double getP() { return m_P; }
    public double getI() { return m_I; }
    public double getD() { return m_D; }
}
