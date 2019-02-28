package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.sensors.CANEncoderPIDSource;
import frc.robot.subsystems.Drive;

/**
 * UseDrive
 */
public class UseDrivePID extends Command {
    private Drive drive;
    private float DEADZONE = 0.05F;
    private float DEADBAND = 0.05F;

    public static final double kDefaultDeadband = 0.02;

    protected double m_deadband = kDefaultDeadband;
  

    PIDController leftFrontController;
    PIDController leftBackController;
    PIDController rightFrontController;
    PIDController rightBackController;
    PIDController dummyPID;

    private double highMaxSpeed = 3500D;
    private double lowMaxSpeed = 1800D;
    private double maxSpeed = highMaxSpeed;
    private double m_rightSideInvertMultiplier = -1.0;

    double kP;
    double kI;
    double kD;
    double kFF = 1 / maxSpeed;


    double leftFrontMotorOutput = 0;
    double rightFrontMotorOutput = 0;
    double leftBackMotorOutput = 0;
    double rightBackMotorOutput = 0;

    private class leftFrontDrivePIDOut implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            leftFrontMotorOutput = output;
        }

    }

    private class rightFrontDrivePIDOutput implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            rightFrontMotorOutput = output;
        }

    }

    private class leftBackDrivePIDOut implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            leftBackMotorOutput = output;
        }

    }

    private class rightBackDrivePIDOutput implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            rightBackMotorOutput = output;
        }

    }

    CANEncoderPIDSource leftFrontSource;
    CANEncoderPIDSource rightFrontSource;
    CANEncoderPIDSource rightBackSource;
    CANEncoderPIDSource leftBackSource;

    public UseDrivePID(Drive _drive) {
        super("Drive");
        drive = _drive;
        requires(drive);

        leftFrontSource = new CANEncoderPIDSource(drive.motors[0].getEncoder());
        rightFrontSource = new CANEncoderPIDSource(drive.motors[1].getEncoder());
        rightBackSource = new CANEncoderPIDSource(drive.motors[3].getEncoder());
        leftBackSource = new CANEncoderPIDSource(drive.motors[2].getEncoder());

        leftFrontSource.setPIDSourceType(PIDSourceType.kRate);
        rightFrontSource.setPIDSourceType(PIDSourceType.kRate);
        leftBackSource.setPIDSourceType(PIDSourceType.kRate);
        rightBackSource.setPIDSourceType(PIDSourceType.kRate);
        
        leftFrontController = new PIDController(kP, kI, kD, leftFrontSource, new leftFrontDrivePIDOut());
        leftBackController = new PIDController(kP, kI, kD, leftBackSource, new leftBackDrivePIDOut());
        rightFrontController = new PIDController(kP, kI, kD, rightFrontSource, new rightFrontDrivePIDOutput());
        rightBackController = new PIDController(kP, kI, kD, rightBackSource, new rightBackDrivePIDOutput());
        
    }

    protected void initialize() {
        leftFrontController.setOutputRange(-1, 1);
        leftBackController.setOutputRange(-1, 1);
        rightFrontController.setOutputRange(-1, 1);
        rightBackController.setOutputRange(-1, 1);

        leftFrontController.setContinuous(false);
        leftBackController.setContinuous(false);
        rightFrontController.setContinuous(false);
        rightBackController.setContinuous(false);

        leftFrontController.setToleranceBuffer(20);
        leftBackController.setToleranceBuffer(20);
        rightFrontController.setToleranceBuffer(20);
        rightBackController.setToleranceBuffer(20);

        leftFrontController.setPercentTolerance(5);
        leftBackController.setPercentTolerance(5);
        rightFrontController.setPercentTolerance(5);
        rightBackController.setPercentTolerance(5);



        leftFrontController.enable();
        leftBackController.enable();
        rightFrontController.enable();
        rightBackController.enable();

        SmartDashboard.putData("lfPID", leftFrontController);
        SmartDashboard.putData("lbPID", leftBackController);
        SmartDashboard.putData("rbPID", rightBackController);
        SmartDashboard.putData("rfPID", rightFrontController);
        
        drive.BrakeMode(IdleMode.kCoast);

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

    protected void execute() {

        SetPIDFromDashboard();

        if(drive.getShift()){
            maxSpeed = lowMaxSpeed;
        }else{
            maxSpeed = highMaxSpeed;
        }

        double x = Robot.oi.getDriverLeftX();
        if (Math.abs(x) > DEADZONE) {
            x = -Math.pow(x, 3);
        } else {
            x = 0;
        }
        double y = Robot.oi.getDriverLeftY();
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

        Vector2d input = new Vector2d(x, y);
        //input.rotate(-drive.gyro.getAngle());

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[MotorType.kFrontLeft.value] = input.x + input.y + rot;
        wheelSpeeds[MotorType.kFrontRight.value] = -input.x + input.y - rot;
        wheelSpeeds[MotorType.kRearLeft.value] = -input.x + input.y + rot;
        wheelSpeeds[MotorType.kRearRight.value] = input.x + input.y - rot;

        normalize(wheelSpeeds);

        leftFrontController.setSetpoint(wheelSpeeds[MotorType.kFrontLeft.value] * maxSpeed);
        rightFrontController
                .setSetpoint(wheelSpeeds[MotorType.kFrontRight.value] * maxSpeed * m_rightSideInvertMultiplier);
        leftBackController.setSetpoint(wheelSpeeds[MotorType.kRearLeft.value] * maxSpeed);
        rightBackController
                .setSetpoint(wheelSpeeds[MotorType.kRearRight.value] * maxSpeed * m_rightSideInvertMultiplier);

        drive.manual(leftFrontMotorOutput, rightFrontMotorOutput, leftBackMotorOutput, rightBackMotorOutput);
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
        PIDController lfPid = (PIDController) SmartDashboard.getData("lfPID");
        PIDController lbPid = (PIDController) SmartDashboard.getData("lbPID");
        PIDController rfPid = (PIDController) SmartDashboard.getData("rfPID");
        PIDController rbPid = (PIDController) SmartDashboard.getData("rbPID");
        
        SmartDashboard.putNumber("LF Rate", drive.motors[0].getEncoder().getVelocity());
        SmartDashboard.putNumber("RF Rate", drive.motors[1].getEncoder().getVelocity());
        SmartDashboard.putNumber("RB Rate", drive.motors[3].getEncoder().getVelocity());
        SmartDashboard.putNumber("LB Rate", drive.motors[2].getEncoder().getVelocity());

        leftFrontController.setPID(lfPid.getP(), lfPid.getI(), lfPid.getD(), lfPid.getF());
        leftBackController.setPID(lbPid.getP(), lbPid.getI(), lbPid.getD(), lbPid.getF());
        rightFrontController.setPID(rfPid.getP(), rfPid.getI(), rfPid.getD(), rfPid.getF());
        rightBackController.setPID(rbPid.getP(), rbPid.getI(), rbPid.getD(), rbPid.getF());
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
}
