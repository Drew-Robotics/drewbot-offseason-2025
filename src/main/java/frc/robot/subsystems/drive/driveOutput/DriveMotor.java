package frc.robot.subsystems.drive.driveOutput;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

public class DriveMotor extends SparkFlex {
    public final int m_CANID;
    public final String m_name;

    public DriveMotor(String name, int CANID) {
        super(CANID, MotorType.kBrushless);

        m_CANID = CANID;
        m_name = name;

        SparkFlexConfig configuration = new SparkFlexConfig();
    }
}
