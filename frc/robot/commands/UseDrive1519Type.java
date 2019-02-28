package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.MotorSafety;
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
public class UseDrive1519Type extends Command {
    private Drive drive;
    private float DEADZONE = 0.05F;
    private float DEADBAND = 0.05F;

    public static final double kDefaultDeadband = 0.02;

    protected double m_deadband = kDefaultDeadband;
  

    CANPIDController leftFrontController;
    CANPIDController leftBackController;
    CANPIDController rightFrontController;
    CANPIDController rightBackController;

    CANPIDController[] controllers;

    PIDController dummyPID;

    private double highMaxSpeed = 3500D;
    private double lowMaxSpeed = 1800D;
    private double maxSpeed = highMaxSpeed;

    private double m_rightSideInvertMultiplier = -1.0;

    double kP;
    double kI;
    double kD;
    double kFF = 1 / maxSpeed;

    public UseDrive1519Type(Drive _drive) {
        super("Drive");
        drive = _drive;
        requires(drive);

        // display PID coefficients on SmartDashboard

        leftFrontController = drive.leftFrontPID;
        rightFrontController = drive.rightFrontPID;
        leftBackController = drive.leftBackPID;
        rightBackController = drive.rightBackPID;
        controllers = new CANPIDController[] {drive.leftFrontPID, drive.rightFrontPID,  drive.leftBackPID, drive.rightBackPID};
        
    }

    protected void initialize() {
        for (int i = 0; i < 4; i ++){
            controllers[i].setOutputRange(-1,1);
        }

        dummyPID = new PIDController(kP, kI, kD, null, null);

        SmartDashboard.putData(dummyPID);


        /*
         * double maxInput = maxSpeed * MAX_SPEED_SCALING_FACTOR;
         * leftFrontController.setInputRange(-(maxInput), maxInput);
         * leftBackController.setInputRange(-(maxInput), maxInput);
         * rightFrontController.setInputRange(-(maxInput), maxInput);
         * rightBackController.setInputRange(-(maxInput), maxInput);
         * 
         * leftFrontController.setContinuous(false);
         * rightFrontController.setContinuous(false);
         * 
         * leftFrontController.setToleranceBuffer(20);
         * rightFrontController.setToleranceBuffer(20);
         * 
         * leftController.setPercentTolerance(5);
         * rightController.setPercentTolerance(5);
         * 
         * leftController.enable(); rightController.enable();
         */
    }

    double leftFrontMotorOutput = 0;
    double rightFrontMotorOutput = 0;
    double leftBackMotorOutput = 0;
    double rightBackMotorOutput = 0;

    protected void execute() {

        SetPIDFromDashboard();

        if(drive.getShift()){
            maxSpeed = lowMaxSpeed;
        }else{
            maxSpeed = highMaxSpeed;
        }


        double x = Robot.oi.getDriverLeftY();
        if (Math.abs(x) > DEADZONE) {
            x = -Math.pow(x, 3);
        } else {
            x = 0;
        }
        double y = Robot.oi.getDriverLeftX();
        if (Math.abs(y) > DEADZONE) {
            y = -Math.pow(y, 3);
        } else {
            y = 0;
        }
        double rot = Robot.oi.getDriverRightX();
        if (Math.abs(rot) > DEADZONE) {
            rot = Math.pow(rot, 3);
        } else {
            rot = 0;
        }

        /* Smooth curving */
        x = x * Math.abs(x);
        y = y * Math.abs(y);
        rot = rot * Math.abs(rot);

        if (Robot.oi.getDriverLeftBumper()) {
            x = 1;
        }

        // drive.mecanum(x, y, rot);

        y = limit(y);
        y = applyDeadband(y, m_deadband);

        x = limit(x);
        x = applyDeadband(x, m_deadband);

        
        Vector2d input = new Vector2d(y, x);
        //input.rotate(-drive.gyro.getAngle());
    
        mecanumDriveCaresian(input.x, input.y, rot);
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
        PIDController pid = (PIDController) SmartDashboard.getData("DrivePID");

        double p = pid.getP();
        double i = pid.getI();
        double d = pid.getD();
        double ff = pid.getF();

        if ((p != kP)) {
            kP = p;
        }
        if ((i != kI)) {
            kI = i;
        }
        if ((d != kD)) {
            kD = d;
        }
        if ((ff != kFF)) {
            kFF = ff;
        }

        leftFrontController.setP(kP);
        rightFrontController.setP(kP);
        leftBackController.setP(kP);
        rightBackController.setP(kP);

        leftFrontController.setI(kI);
        rightFrontController.setI(kI);
        leftBackController.setI(kI);
        rightBackController.setI(kI);

        leftFrontController.setD(kD);
        rightFrontController.setD(kD);
        leftBackController.setD(kD);
        rightBackController.setD(kD);

        leftFrontController.setFF(kFF);
        rightFrontController.setFF(kFF);
        leftBackController.setFF(kFF);
        rightBackController.setFF(kFF);
    }

        /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    protected double limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    protected double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public void mecanumDriveCaresian(double xIn,double yIn, double rot){
        double[] wheelSpeeds = new double[4];
        wheelSpeeds[MotorType.kFrontLeft.value] = xIn + yIn + rot;
        wheelSpeeds[MotorType.kFrontRight.value] = -xIn + yIn - rot;
        wheelSpeeds[MotorType.kRearLeft.value] = -xIn + yIn + rot;
        wheelSpeeds[MotorType.kRearRight.value] = xIn + yIn - rot;

        normalize(wheelSpeeds);

        leftFrontMotorOutput = wheelSpeeds[MotorType.kFrontLeft.value];
        rightFrontMotorOutput = wheelSpeeds[MotorType.kFrontRight.value];
        leftBackMotorOutput = wheelSpeeds[MotorType.kRearLeft.value];
        rightBackMotorOutput = wheelSpeeds[MotorType.kRearRight.value];

        leftFrontController.setReference(leftFrontMotorOutput * maxSpeed, ControlType.kVelocity);
        rightFrontController.setReference(rightFrontMotorOutput
                 * maxSpeed * m_rightSideInvertMultiplier,
                ControlType.kVelocity);
        leftBackController.setReference(leftBackMotorOutput * maxSpeed, ControlType.kVelocity);
        rightBackController.setReference(rightBackMotorOutput
                 * maxSpeed * m_rightSideInvertMultiplier,
                ControlType.kVelocity);

        drive.manual(leftFrontMotorOutput, rightFrontMotorOutput, leftBackMotorOutput, rightBackMotorOutput);
    }
    
}
