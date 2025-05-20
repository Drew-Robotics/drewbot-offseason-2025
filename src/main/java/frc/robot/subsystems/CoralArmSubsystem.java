package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.ArmFeedforward;

import frc.robot.constants.CoralConstants;

public class CoralArmSubsystem extends Subsystem {
    private final SparkFlex m_armMotor;
    private SparkMax m_armEncoderController;
    private SparkAbsoluteEncoder m_armEncoder;
    private PIDController m_pidControllerBaby;
    private ArmFeedforward m_armFeedForward;

    public CoralArmSubsystem() {
        m_armMotor = new SparkFlex(CoralConstants.kCoralArm, MotorType.kBrushless);
        m_armEncoderController = new SparkMax(CoralConstants.kCoralArmEncoder, MotorType.kBrushless);
        m_armEncoder = m_armEncoderController.getAbsoluteEncoder();
        m_pidControllerBaby = new PIDController(
            CoralConstants.kP, 
            CoralConstants.kI, 
            CoralConstants.kD);
        m_armFeedForward = new ArmFeedforward(
            0,
            CoralConstants.kG,
            0
        );
    }

    public void rotate(Rotation2d desiredAngle){
        Rotation2d currentAngle = Rotation2d.fromRadians(m_armEncoder.getPosition()).minus(Rotation2d.fromDegrees(63.42711));

        double pidCalculatedVoltage = m_pidControllerBaby.calculate(currentAngle.getRadians(), desiredAngle.getRadians());  
        double feedForwardVoltage = m_armFeedForward.calculate(currentAngle.getRadians(), 0);
        double voltage = pidCalculatedVoltage + feedForwardVoltage;

        voltage = MathUtil.clamp(voltage, -12, 12);
        m_armMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}
}



/*
        Oh say, can you see
        By the dawn's early light
        What so proudly we hailed
        At the twilight's last gleaming?
        Whose broad stripes and bright stars
        Through the perilous fight
        O'er the ramparts we watched
        Were so gallantly, yeah, streaming?
        And the rockets' red glare
        The bombs bursting in air
        Gave proof through the night
        That our flag was still there
        O say, does that star-spangled banner yet wave
        O'er the land of the free and the home of the brave
        * CAAAAAW WHAT THE FRICKITY FRICK FRACK IS A KILOMETER
        * \\             //
        *  \\\' ,      / //
        *   \\\//,   _/ //,
        *    \_-//' /  //,
        *      \ ///  //`
        *     /  ;  \\\`__/_
        *    /,)-^ _\` \\\
        *    (/   \\ //\\
        *        // _//\\\\
        *       ((` ((
        */