package frc.robot.subsystems.drive.driveOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

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
    }

    public void setAngle(Angle angle) {
        m_closedLoop.setReference(angle.in(Units.Radians), ControlType.kPosition);
    }

    public Angle getAngle() {
        return Units.Radians.of(m_encoder.getPosition());
    }
}

