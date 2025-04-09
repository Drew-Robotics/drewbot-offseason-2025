package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    protected final NetworkTable m_table;

    protected abstract void publishInit();
    protected abstract void publishPeriodic();

    protected Subsystem() {
        String name = this.getClass().getSimpleName();
        m_table = NetworkTableInstance.getDefault().getTable(name);

        publishInit();
    }

    @Override
    public void periodic() {
        super.periodic();

        publishPeriodic();
    } 
}
