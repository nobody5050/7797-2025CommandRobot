// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OpSide extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param Drivebase
   * @param m_intake 
   * @param m_Arm The drivetrain subsystem on which this command will run
   */
  public OpSide(SwerveSubsystem Drivebase, Arm m_Arm, Intake m_intake) {
    addCommands(
      Drivebase.getAutonomousCommand("Copy of OpAngle"),
      new AutoArmRun(m_Arm, 1.6),
      new InstantCommand(() -> m_Arm.ArmRun(Constants.kStopSpeed)),
      new AutoIntakeRun(m_intake, 3));
      new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed));
   
  }
}