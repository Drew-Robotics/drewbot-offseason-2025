package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ElevatorConstants;
import frc.robot.motorconfigs.MotorConfigTools;

public class ElevatorSubsystem extends Subsystem {
    private final SparkMax m_elevatorMotorLeft;
    private final SparkMax m_elevatorMotorRight; // Follows Left Motor

    private final SparkClosedLoopController m_elevatorClosedLoop;

    private final RelativeEncoder m_elevatorEncoderLeft; // TODO: Not sure if it is relative or absolute

    private static ElevatorSubsystem m_instance;
    public static ElevatorSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new ElevatorSubsystem();
        
        return m_instance;
    }

    public ElevatorSubsystem() {
        super();

        m_elevatorMotorLeft = new SparkMax(ElevatorConstants.CANIDs.kLeft, MotorType.kBrushless);
        m_elevatorMotorRight = new SparkMax(ElevatorConstants.CANIDs.kRight, MotorType.kBrushless);

        m_elevatorClosedLoop = m_elevatorMotorLeft.getClosedLoopController();
        
        SparkMaxConfig m_elevatorMotorConfigLeft = new SparkMaxConfig();
        SparkMaxConfig m_elevatorMotorConfigRight = new SparkMaxConfig();

        m_elevatorEncoderLeft = m_elevatorMotorLeft.getEncoder();

        m_elevatorMotorConfigLeft
            .apply(MotorConfigTools.mkMotorConfig(IdleMode.kCoast, ElevatorConstants.kCurrentLimit))
            .apply(MotorConfigTools.mkEncoderConfig(ElevatorConstants.ConversionFactor.kConversions)) 
            .apply(MotorConfigTools.mkClosedLoopConfig(
                FeedbackSensor.kPrimaryEncoder, 
                ElevatorConstants.ClosedLoop.kPID, 
                ElevatorConstants.ClosedLoop.outputRange
            ));

        m_elevatorMotorConfigRight
            .apply(MotorConfigTools.mkMotorConfig(IdleMode.kCoast, ElevatorConstants.kCurrentLimit))
            .follow(m_elevatorMotorLeft); // FOLLOWS LEFT MOTOR

        m_elevatorMotorLeft.configure(m_elevatorMotorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevatorMotorRight.configure(m_elevatorMotorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPosition(Distance position) {
        m_elevatorClosedLoop.setReference(position.in(Meters), ControlType.kPosition);
    }

    public Distance getPosition() {
        return Meters.of(m_elevatorEncoderLeft.getPosition());
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(m_elevatorEncoderLeft.getVelocity());
    }

    /* OVERRIDES */

    // Logging

    protected void publishInit() {}

    protected void publishPeriodic() {
        SmartDashboard.putNumber("Elevator Position (m)", getPosition().in(Units.Meters));
        SmartDashboard.putNumber("Elevator Velocity (mps)", getVelocity().in(Units.MetersPerSecond));
    }
}
