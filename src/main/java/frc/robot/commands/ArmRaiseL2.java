package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmRaiseL2 extends Command {
    private Arm m_subsystem;

    /** Creates a new InnerArmRaiseM. */
    public ArmRaiseL2(Arm subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("RaiseDone", false);
        m_subsystem.raiseL2();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("RaiseDone", true);
        m_subsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_subsystem.isAtSetPoint();
    }
}