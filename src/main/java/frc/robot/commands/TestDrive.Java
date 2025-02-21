// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Arm;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Commands.AutoIntakeRun;

public class TestDrive extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param Drivebase
   * @param m_intake 
   * @param m_Arm The drivetrain subsystem on which this command will run
   */
  public TestDrive(SwerveSubsystem Drivebase, Shooter m_shooter, Intake m_intake) {
    addCommands(
      new getAutonomousCommand("Angular"),
      new AutoArmRun(m_Arm,2),
      new AutoIntakeRun(m_intake, 1));
  }
}