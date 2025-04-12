package frc.robot.subsystems.drive.drivecomponents;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

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

    public void setAngle(Rotation2d angle) {
        m_closedLoop.setReference(angle.getRadians(), ControlType.kPosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_encoder.getPosition());
    }
}

