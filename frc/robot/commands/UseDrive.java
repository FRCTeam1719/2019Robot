    package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drive;
/**
 * UseDrive
 */
public class UseDrive extends Command {
    private Drive drive;
/**
 * Simple drive
 * @param _drive
 */
    public UseDrive(Drive _drive) {
        super("Drive");
        drive = _drive;
        requires(drive);
        SmartDashboard.putNumber("Strafe correction: ", 0.115);

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
        double strafeCorrect = SmartDashboard.getNumber("Strafe correction: ", .115);
        x = x * (1 + strafeCorrect);

        if(drive.getShift()){
            x = x / 4;
            y = y / 4;
        }

        drive.mecanum(x, y, rot);

        /*SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("rot", rot);

        Vector2d input = new Vector2d(y, x);
        //input.rotate(-drive.gyro.getAngle());

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[MotorType.kFrontLeft.value] = input.x + input.y + rot;
        wheelSpeeds[MotorType.kFrontRight.value] = -input.x + input.y - rot;
        wheelSpeeds[MotorType.kRearLeft.value] = -input.x + input.y + rot;
        wheelSpeeds[MotorType.kRearRight.value] = input.x + input.y - rot;

        normalize(wheelSpeeds);
        drive.manual(wheelSpeeds[MotorType.kFrontLeft.value],
                     wheelSpeeds[MotorType.kFrontRight.value] * (-1),
                     wheelSpeeds[MotorType.kRearLeft.value],
                     wheelSpeeds[MotorType.kRearRight.value] * (-1));
    */ }

    protected boolean isFinished() {
        return false;
    }

    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }
}