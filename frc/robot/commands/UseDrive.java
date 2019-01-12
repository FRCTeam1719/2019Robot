package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drive;

/**
 * UseDrive
 */
public class UseDrive extends Command {
    private Drive drive;

    public UseDrive(Drive _drive) {
        super("Drive");
        drive = _drive;
        requires(drive);

    }

    protected void initialize() {

    }

    protected void execute() {
        double x = -Robot.oi.getDriverLeftX();
        double y = -Robot.oi.getDriverLeftY();
        double rot = Robot.oi.getDriverRightX();

        /* Smooth curving */
        x = x * Math.abs(x);
        y = y * Math.abs(y);
        rot = rot * Math.abs(rot);

        drive.mecanum(x, y, rot);
    }

    protected boolean isFinished() {
        return false;
    }
}