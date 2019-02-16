package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

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
public class UseDrivePID extends Command {
    private Drive drive;
    private float DEADZONE = 0.05F;
    private float DEADBAND = 0.05F;

    PIDController leftFrontController;
    PIDController leftBackController;
    PIDController rightFrontController;
    PIDController rightBackController;

    private double maxSpeed = 4000D;
    private double m_rightSideInvertMultiplier = -1.0;

    double kP;
    double kI;
    double kD;
    double kFF = 1 / maxSpeed;

    public UseDrivePID(Drive _drive) {
        super("Drive");
        drive = _drive;
        requires(drive);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Drive kP", 0);
        SmartDashboard.putNumber("Drive kI", 0);
        SmartDashboard.putNumber("Drive kD", 0);
        SmartDashboard.putNumber("Drive Feed Forward", 0);
        SmartDashboard.putNumber("Drive Target speed", 0);
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

    protected void execute() {

        SetPIDFromDashboard();

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

        // Compensate for gyro angle.
        if (x > DEADBAND) {
            x = 0;
        }
        if (y > DEADBAND) {
            x = 0;
        }
        Vector2d input = new Vector2d(y, x);
        input.rotate(-drive.gyro.getAngle());

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
        double p = SmartDashboard.getNumber("Drive kP", 0);
        double i = SmartDashboard.getNumber("Drive kI", 0);
        double d = SmartDashboard.getNumber("Drive kD", 0);
        double ff = SmartDashboard.getNumber("Drive Feed Forward", 0);

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


        leftFrontController.setPID(kP, kI, kD, kFF);
        rightFrontController.setPID(kP, kI, kD, kFF);
        leftBackController.setPID(kP, kI, kD, kFF);
        rightBackController.setPID(kP, kI, kD, kFF);
    }
}
