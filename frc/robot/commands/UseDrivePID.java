package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drive;

/**
 * UseDrive
 */
public class UseDrivePID extends Command {
    private Drive drive;
    private float DEADZONE = 0.05F;


    double kP;
    double kI;
    double kD;
    double kFF;
    double rng;
    public UseDrivePID(Drive _drive) {
        super("Drive");
        drive = _drive;
        requires(drive);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Arm steady kP", 0);
        SmartDashboard.putNumber("Arm steady kI", 0);
        SmartDashboard.putNumber("Arm steady kD", 0);
        SmartDashboard.putNumber("Feed Forward", 0);
        SmartDashboard.putNumber("Target speed", 0);
    }

    protected void initialize() {

    }

    protected void execute() {
        double p = SmartDashboard.getNumber("Arm steady kP", 0);
        double i = SmartDashboard.getNumber("Arm steady kI", 0);
        double d = SmartDashboard.getNumber("Arm steady kD", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double _rng = SmartDashboard.getNumber("Arm steady integral range", 0);

        if((p != kP)) { drive.setP(p); kP = p; }
        if((i != kI)) { drive.setI(i); kI = i; }
        if((d != kD)) { drive.setD(d); kD = d; }
        //if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }

        double x = Robot.oi.getDriverLeftX();
        if (x > DEADZONE) {
            x = -Math.pow(x, 3);
        } else {
            x = 0;
        }
        double y = Robot.oi.getDriverLeftY();
        if (y > DEADZONE) {
            y = -Math.pow(y, 3);
        } else {
            y = 0;
        }
        double rot = Robot.oi.getDriverRightX();
        if (rot > DEADZONE) {
            rot = Math.pow(rot, 3);
        } else {
            rot = 0;
        }

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