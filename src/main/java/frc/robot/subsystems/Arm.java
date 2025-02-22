// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private SparkMax m_armMotor;
  private SparkClosedLoopController m_pidController;
  private AbsoluteEncoder m_encoder;
  private double m_setPoint;
  private SparkMaxConfig motorConfig;

  /** Creates a new OuterArm. */
  public Arm() {
    m_armMotor = new SparkMax(Constants.kArmMotorPort, MotorType.kBrushless);
    m_pidController = m_armMotor.getClosedLoopController();
    m_encoder = m_armMotor.getAbsoluteEncoder();
    motorConfig = new SparkMaxConfig();

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAnalogSensor)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.kP)
        .i(Constants.kI)
        .d(Constants.kD)
        .outputRange(Constants.kMin, Constants.kMax)
        .iZone(Constants.kIZone);

    m_armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void raise(){
    m_armMotor.set(Constants.raiseSpeed);
  }
  public void raiseL1() {
    m_pidController.setReference(Constants.upPIDReferenceL1, SparkMax.ControlType.kPosition);
    m_setPoint = Constants.upPIDReferenceL2;
  }

  public void raiseL2() {
    m_pidController.setReference(Constants.upPIDReferenceL2, SparkMax.ControlType.kPosition);
    m_setPoint = Constants.upPIDReferenceL2;
  }
  public void raiseL3() {
    m_pidController.setReference(Constants.upPIDReferenceL3, SparkMax.ControlType.kPosition);
    m_setPoint = Constants.upPIDReferenceL3;
  }
  public void raiseL4() {
    m_pidController.setReference(Constants.upPIDReferenceL4, SparkMax.ControlType.kPosition);
    m_setPoint = Constants.upPIDReferenceL4;
  }
  public void raiseFeed() {
    m_pidController.setReference(Constants.upPIDReferenceF, SparkMax.ControlType.kPosition);
    m_setPoint = Constants.upPIDReferenceF;
  }
  public void lower(){
    m_armMotor.set(Constants.lowerSpeed);
  }
  public void lowerPID() {
    m_pidController.setReference(Constants.downPIDReference, SparkMax.ControlType.kPosition);
    m_setPoint = Constants.downPIDReference; 
  }
  public void stop(){
    m_armMotor.set(Constants.kStopSpeed);
  }
  public boolean isAtSetPoint() {
    return (Math.abs(m_setPoint - Units.rotationsToDegrees(m_encoder.getPosition())) <= Constants.outerPIDTolorence);
  }
  public boolean isAtStowed() {
    return (Math.abs(Constants.downPIDReference - Units.rotationsToDegrees(m_encoder.getPosition())) <= Constants.outerPIDTolorence);
  }

  public void ArmRun(double speed) {
    m_armMotor.set(speed);
  }

  public void raiseWithInput(double speed) {
    m_armMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("outer arm", m_encoder.getPosition());
    // This method will be called once per scheduler run
  }
}