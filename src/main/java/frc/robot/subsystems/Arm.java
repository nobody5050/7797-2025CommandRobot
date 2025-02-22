package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import java.util.List;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
  SparkMax m_Arm;
  /*
   * private SparkClosedLoopController m_ArmPID;
   * private RelativeEncoder m_encoder;
   * private SparkMaxConfig m_ArmConfig;
   */
  /*
   * public enum ArmPivotPositions {
   * L1(0.2),
   * L2(0),
   * L3(0),
   * L4(0);
   * 
   * private final double value;
   * 
   * ArmPivotPositions(double value) {
   * this.value = value;
   * }
   * 
   * public double getValue() {
   * return value;
   * }
   * 
   * }
   */

  public Arm() {
    /** Creates a new Intake. */
    m_Arm = new SparkMax(Constants.kArmID, MotorType.kBrushless);
    // m_ArmConfig = new SparkMaxConfig();
    /*
     * m_encoder = m_Arm.getEncoder();
     * m_ArmPID = m_Arm.getClosedLoopController();
     * 
     * m_ArmConfig.closedLoop
     * .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
     * .p(4)
     * .d(0)
     * .outputRange(-.5, .5);
     * m_ArmConfig
     * .smartCurrentLimit(40)
     * .idleMode(IdleMode.kBrake);
     * m_Arm.configure(m_ArmConfig, ResetMode.kResetSafeParameters,
     * PersistMode.kPersistParameters);
     */
  }

  public void ArmRun(double speed) {
    // set motor speed to intake, eject or stop
    m_Arm.set(speed);

    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     * 
     * SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
     */
  }
  /*
   * public double getPivotPosition() {
   * return m_encoder.getPosition();
   * }
   */

  /*
   * public Command RunArm(double speed) {
   * return ArmRun();
   */

  /*
   * public void pidSetPosition(ArmPivotPositions position) {
   * m_ArmPID.setReference(position.getValue(), ControlType.kPosition);
   * }
   */

}
