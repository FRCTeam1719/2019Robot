    package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
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
public class UseDriveRotPID extends Command {
    private Drive drive;

    private PIDController anglePID;
    private double Kp, Ki, Kd = 0;
    private double TOLERANCE = 5;

    private double stickTolerance = .1;
    double pidOut;

    private class AnglePIDOUT implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			pidOut = output;
		}
    	
}

    public UseDriveRotPID(Drive _drive) {
        super("Drive");
        drive = _drive;
        requires(drive);
        SmartDashboard.putNumber("Strafe correction: ", 0.115);
        drive.gyro.setPIDSourceType(PIDSourceType.kDisplacement);
        anglePID = new PIDController(Kp, Ki, Kd, drive.gyro, new AnglePIDOUT());
        SmartDashboard.putData(anglePID);
    }

    protected void initialize() {
        anglePID.setOutputRange(-1D, 1D);
        anglePID.setSetpoint(drive.gyro.getAngle());
        anglePID.setPercentTolerance(TOLERANCE);
        anglePID.enable();
        SmartDashboard.putData("Angle PID", anglePID);
    }

    protected void execute() {
        SetPIDFromDashboard();
        double x = -Robot.oi.getDriverLeftX();
        double y = -Robot.oi.getDriverLeftY();
        double rot = Robot.oi.getDriverRightX();
        if(Math.abs(rot) > stickTolerance){
            //Manual rotation
            anglePID.disable();
            anglePID.setSetpoint(drive.gyro.getAngle());
            rot = rot * Math.abs(rot);
        }else{
            //Else pid take the wheel
            anglePID.enable();
            rot = pidOut;
        }


        /* Smooth curving */
        x = x * Math.abs(x);
        y = y * Math.abs(y);
        
        double strafeCorrect = SmartDashboard.getNumber("Strafe correction: ", .115);
        x = x * (1 + strafeCorrect);
        drive.mecanum(x, y, rot);
    }

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

    protected void SetPIDFromDashboard() {
        anglePID = (PIDController)SmartDashboard.getData("Angle PID");
      }
}