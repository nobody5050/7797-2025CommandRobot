package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class Climb extends SubsystemBase {
  SparkMax m_climb;

  public Climb() {
  /** Creates a new Intake. */
    m_climb = new SparkMax(Constants.kClimbID, MotorType.kBrushless);
    
  }
  
  public void ClimbRun(double speed) {
  //set motor speed  to intake, eject or stop
    m_climb.set(speed);

  }
}
