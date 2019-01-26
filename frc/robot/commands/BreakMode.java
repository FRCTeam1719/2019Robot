package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drive;

public class BreakMode extends Command {
    Drive drive;
    IdleMode idleMode;

    public BreakMode(Drive _drive, IdleMode _idleMode){
        drive = _drive;
        idleMode = _idleMode;
    }
	@Override
    protected void initialize() {
        drive.motors[0].setIdleMode(idleMode);
        drive.motors[1].setIdleMode(idleMode);
        drive.motors[2].setIdleMode(idleMode);
        drive.motors[3].setIdleMode(idleMode);
    }
  
    @Override
    protected boolean isFinished() {
        return true;
    }
}