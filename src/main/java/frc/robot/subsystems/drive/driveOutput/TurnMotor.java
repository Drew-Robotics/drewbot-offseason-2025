package frc.robot.subsystems.drive.driveOutput;

import com.revrobotics.spark.SparkMax;

public class TurnMotor extends SparkMax {
    public final int m_CANID;
    public final String m_name;

    public TurnMotor(String name, int CANID) {
        super(CANID, MotorType.kBrushless);

        m_CANID = CANID;
        m_name = name;
    }
}

