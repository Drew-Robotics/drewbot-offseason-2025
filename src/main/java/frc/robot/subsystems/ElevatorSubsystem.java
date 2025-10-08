package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.ClosedLoop;

import com.revrobotics.spark.SparkMax;


public class ElevatorSubsystem extends Subsystem {
    private final SparkFlex m_elevatorMotorLeft;
    private final SparkFlex m_elevatorMotorRight;
    
    private final RelativeEncoder m_elevatorEncoderLeft;
    private final RelativeEncoder m_elevatorEncoderRight;

    private final SparkClosedLoopController m_closedLoop;

    public ElevatorSubsystem() {

        m_elevatorMotorLeft = new SparkFlex(ElevatorConstants.CANIDs.kLeft, MotorType.kBrushless);
        m_elevatorMotorRight = new SparkFlex(ElevatorConstants.CANIDs.kRight, MotorType.kBrushless);

        m_elevatorEncoderLeft = m_elevatorMotorLeft.getEncoder();
        m_elevatorEncoderRight = m_elevatorMotorRight.getEncoder();
        m_closedLoop = m_elevatorMotorLeft.getClosedLoopController();

        SparkFlexConfig configurationLeft = new SparkFlexConfig();
        SparkFlexConfig configurationRight = new SparkFlexConfig();
        configurationLeft
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) ElevatorConstants.kCurrentLimit.in(Units.Amps));



    }

    protected void publishInit() {}

    protected void publishPeriodic() {}
}

