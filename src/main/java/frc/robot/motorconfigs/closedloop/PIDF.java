package frc.robot.motorconfigs.closedloop;

public class PIDF extends PID {
    public final double m_FF;

    public PIDF(double P, double I, double D, double FF) {
        super(P, I, D);

        m_FF = FF;
    }

    public double getFF() { return m_FF; }
}
