package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.revrobotics.spark.SparkMax;

import java.util.List;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
  SparkMax m_Arm;
  

  public Arm() {
  /** Creates a new Intake. */
    m_Arm = new SparkMax(Constants.kArmID, MotorType.kBrushless);
    
  }
  
  public void ArmRun(double speed) {
  //set motor speed  to intake, eject or stop
    m_Arm.set(-speed);
   
  }
  /*public Command RunArm(double speed) {
      return ArmRun();*/
      //please help
    }

  
 

 