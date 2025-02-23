package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.commands.ArmRaiseL1;
import frc.robot.commands.ArmRaiseL2;
import frc.robot.commands.ArmRaiseL3;
import frc.robot.commands.ArmRaiseL4;
import frc.robot.commands.OpSide;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/kraken"));
  final CommandXboxController driverXbox = new CommandXboxController(0);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true);

  public RobotContainer() {
    NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> m_intake.IntakeRun(Constants.kIntakeOut)));
    NamedCommands.registerCommand("RunArm", Commands.runOnce(() -> new ArmRaiseL1(m_arm)));
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(
          () -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.b().whileTrue(drivebase.driveToPose(
          new Pose2d(new Translation2d(0.25, 0), Rotation2d.fromDegrees(0))));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper()
          .onTrue(new InstantCommand(() -> m_intake.IntakeRun(Constants.kIntakeOut)))
          .onFalse(new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed)));
      driverXbox.rightBumper()
          .onTrue(new InstantCommand(() -> m_intake.IntakeRun(Constants.kIntakeIn)))
          .onFalse(new InstantCommand(() -> m_intake.IntakeRun(Constants.kStopSpeed)));
      driverXbox.leftTrigger()
          .onTrue(new InstantCommand(() -> m_arm.ArmRun(Constants.kArmOut)))
          .onFalse(new InstantCommand(() -> m_arm.ArmRun(Constants.kStopSpeed)));
      driverXbox.rightTrigger()
          .onTrue(new InstantCommand(() -> m_arm.ArmRun(Constants.kArmIn)))
          .onFalse(new InstantCommand(() -> m_arm.ArmRun(Constants.kStopSpeed)));

      driverXbox.x().whileTrue(new ArmRaiseL1(m_arm));
      driverXbox.y().whileTrue(new ArmRaiseL2(m_arm));
      driverXbox.b().whileTrue(new ArmRaiseL3(m_arm));
      driverXbox.a().whileTrue(new ArmRaiseL4(m_arm));
    }
  }

  public Command getAutonomousCommand() {
    return new OpSide(drivebase, m_arm, m_intake);
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
