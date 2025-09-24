package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.constants.L1CoralConstants;

public class L1CoralArmSubsystem extends Subsystem{
    private final SparkFlex m_armMotor;
    private final SparkFlex m_handMotor;
    private final SparkFlex m_correctionMotor;

    private final SparkClosedLoopController m_armClosedLoop;
    private final SparkClosedLoopController m_handClosedLoop;
    private final SparkClosedLoopController m_correctionMotorClosedLoop;

    public L1CoralArmSubsystem() {
        m_armMotor = new SparkFlex(L1CoralConstants.CANIDs.kCoralArm, MotorType.kBrushless);
        m_handMotor = new SparkFlex(L1CoralConstants.CANIDs.kCoralHand, MotorType.kBrushless);
        m_correctionMotor = new SparkFlex(L1CoralConstants.CANIDs.kCoralCorrection, MotorType.kBrushless);

        m_armClosedLoop = m_armMotor.getClosedLoopController();
        m_handClosedLoop = m_armMotor.getClosedLoopController();
        m_correctionMotorClosedLoop = m_correctionMotor.getClosedLoopController();


    }

// Logging

    protected void publishInit() {

    }

    protected void publishPeriodic() {
    }

    

}
