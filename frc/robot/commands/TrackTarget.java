    package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
public class TrackTarget extends Command {
    private Drive drive;

    private PIDController anglePID;
    private double Kp, Ki, Kd = 0;
    private double TOLERANCE = 10;

    private double stickTolerance = .01;
    double pidOut;

    double yawOff = 0;

    NetworkTable chickenVision;

    private class AnglePIDOUT implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			pidOut = output;
		}
    	
}

    public TrackTarget(Drive _drive) {
        super("Drive");
        drive = _drive;
        requires(drive);
        SmartDashboard.putNumber("Strafe correction: ", 0.115);
        drive.gyro.setPIDSourceType(PIDSourceType.kDisplacement);
        anglePID = new PIDController(Kp, Ki, Kd, drive.gyro, new AnglePIDOUT());
        SmartDashboard.putData(anglePID);
       

    }
    NetworkTableEntry yawEntry, tapeDetected;
    NetworkTableInstance instance;

    protected void initialize() {
        instance = NetworkTableInstance.getDefault();
 
        chickenVision = instance.getTable("ChickenVision");
 
tapeDetected = chickenVision.getEntry("tapeDetected");
yawEntry = chickenVision.getEntry("tapeYaw");        
anglePID.setOutputRange(-.5D, .5D);
        anglePID.setSetpoint(drive.gyro.getAngle());
        anglePID.setAbsoluteTolerance(TOLERANCE);
        anglePID.enable();
        SmartDashboard.putData("Angle PID", anglePID);
    }
    protected void execute() {
        SetPIDFromDashboard();
        double x = -Robot.oi.getDriverLeftX();
        double y = -Robot.oi.getDriverLeftY();
        double rot = Robot.oi.getDriverRightX();
        boolean tapeSeen = tapeDetected.getBoolean(false);
        yawOff = yawEntry.getDouble(0);
        
        SmartDashboard.putNumber("dumb", yawOff);
        if(tapeSeen){
            anglePID.setSetpoint(drive.gyro.getAngle() + yawOff);
            rot = pidOut;
        }else{
            anglePID.setSetpoint(drive.gyro.getAngle());
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