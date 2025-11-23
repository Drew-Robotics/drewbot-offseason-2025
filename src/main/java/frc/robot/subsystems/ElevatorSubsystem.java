package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class ElevatorSubsystem extends Subsystem {
    private final SparkMax m_elevatorMotorLeft;
    private final SparkMax m_elevatorMotorRight;
    
    private final RelativeEncoder m_elevatorEncoderLeft;
    private final RelativeEncoder m_elevatorEncoderRight;

    private final SparkClosedLoopController m_closedLoop;

    public ElevatorSubsystem() {

        m_elevatorMotorLeft = new SparkMax(ElevatorConstants.CANIDs.kLeft, MotorType.kBrushless);
        m_elevatorMotorRight = new SparkMax(ElevatorConstants.CANIDs.kRight, MotorType.kBrushless);

        m_elevatorEncoderLeft = m_elevatorMotorLeft.getEncoder();
        m_elevatorEncoderRight = m_elevatorMotorRight.getEncoder();
        m_closedLoop = m_elevatorMotorLeft.getClosedLoopController();

        SparkMaxConfig configurationLeft = new SparkMaxConfig();
        SparkMaxConfig configurationRight = new SparkMaxConfig();

        configurationLeft
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) ElevatorConstants.kCurrentLimit.in(Units.Amps));
        configurationLeft.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        configurationLeft.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                ElevatorConstants.PID.kP,
                ElevatorConstants.PID.kI,
                ElevatorConstants.PID.kD,
                ElevatorConstants.PID.kFF
            );
        
        configurationRight.follow(ElevatorConstants.CANIDs.kLeft);

        m_elevatorMotorLeft.configure(configurationLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevatorMotorRight.configure(configurationRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Distance getHeight() {
        return rotationsToHeight(m_elevatorEncoderLeft.getPosition());
    }

    public Distance rotationsToHeight(Double r){
        //Change into a normal unit conversion like you wanted to at the beginning of this train wreck
        Distance rotationsToDistance = Units.Meters.of(
            (ElevatorConstants.kMaxHeight.in(Units.Meters)/ElevatorConstants.kMaxRotations.in(Units.Rotations))
            *r
        );


        // function for if it were a non-linear conversion:

        // Distance rotationsToDistance = Units.Meters.of(
        //     ((ElevatorConstants.kMaxHeight.in(Meters) - ElevatorConstants.kMinHeight.in(Meters))
        //     /(ElevatorConstants.kMaxRotations.in(Units.Rotations) - ElevatorConstants.kMinRotations.in(Units.Rotations)))
        //     *r

        //     +(
        //     ElevatorConstants.kMinHeight.in(Units.Meters)

        //     -(ElevatorConstants.kMinRotations.in(Units.Rotations)

        //     *(ElevatorConstants.kMaxHeight.in(Meters) - ElevatorConstants.kMinHeight.in(Meters))
        //     /(ElevatorConstants.kMaxRotations.in(Units.Rotations) - ElevatorConstants.kMinRotations.in(Units.Rotations))))
        // );

        return rotationsToDistance;
    }

    public Rotation2d heightToRotations (Distance dist){
        
        Angle rots = Units.Rotations.of(
            (ElevatorConstants.kMaxRotations.in(Units.Rotations)/ElevatorConstants.kMaxHeight.in(Units.Meters))
            *dist.in(Units.Meters)
        );

        return Rotation2d.fromRadians(rots.in(Units.Radians));
    }

    public LinearVelocity velocityCalculation(Distance pos0, Distance pos1, double time){
        LinearVelocity v = Units.MetersPerSecond.of((pos0.in(Units.Meters) - pos1.in(Units.Meters))/time);
        return v;
    }

    protected void publishInit() {}

    protected void publishPeriodic() {}

    @Override
    public void periodic() {

    }
}