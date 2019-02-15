/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.commands.UseDrive;
import jdk.jfr.Percentage;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Drive Subsystem for Mechanom Drive
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final int FRONT_LEFT = 0;
  private final int FRONT_RIGHT = 1;
  private final int BACK_LEFT = 2;
  private final int BACK_RIGHT = 3;
  public final AHRS navX;
  public final AnalogGyro gyro;
  public CANSparkMax[] motors;

  DigitalInput leftSensor;
  DigitalInput centerSensor;
  DigitalInput rightSensor;

  private MecanumDrive robotDrive;

  // PID things
  float P, I, D = 1;
  float integral, previous_error, setpoint, error, derivative, rcw = 0;
  public CANPIDController leftFrontPID;
  public CANPIDController rightFrontPID;
  public CANPIDController leftBackPID;
  public CANPIDController rightBackPID;

  public CANEncoder leftFrontEncoder;
  public CANEncoder rightFrontEncoder;
  public CANEncoder leftBackEncoder;
  public CANEncoder rightBackEncoder;

  public Drive(CANSparkMax leftFrontMotor, CANSparkMax rightFrontMotor, CANSparkMax leftBackMotor,
      CANSparkMax rightBackMotor, AHRS _navX, DigitalInput _leftSensor, DigitalInput _centerSensor,
      DigitalInput _rightSensor, AnalogGyro _gyro) {
    super();
    navX = _navX;
    // 1D array of the speed controllers passed
    motors = new CANSparkMax[] { leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor };

    // makes a new general robot drive that is a mecanum drive with front and back
    // motors. wpilib's mecanum code.
    robotDrive = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

    leftSensor = _leftSensor;
    centerSensor = _centerSensor;
    rightSensor = _rightSensor;

    gyro = _gyro;

    leftFrontPID = new CANPIDController(motors[0]);
    rightFrontPID = new CANPIDController(motors[1]);
    leftBackPID = new CANPIDController(motors[2]);
    rightBackPID = new CANPIDController(motors[3]);

    leftFrontEncoder = motors[0].getEncoder();
    rightFrontEncoder = motors[1].getEncoder();
    leftBackEncoder = motors[2].getEncoder();
    rightBackEncoder = motors[3].getEncoder();

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new UseDrive(this));
  }

  /**
   * Returns the motor speeds for a given mecanum-style input
   * 
   * @param x   the x position of the right stick
   * @param y   the y position of the right stick
   * @param rot the x position of the left stick
   * @return an array of motor speeds ([x][y])
   */
  public void mecanum(double x, double y, double rot) {
    robotDrive.driveCartesian(-x, y, rot, gyro.getAngle());
    // System.out.println("Dr" + gyro.getAngle());

    for (CANSparkMax motor : motors) {
      System.out.println(motor.get());
    }
  }

  public void setAngleSetpoint(float angle) {
    this.setpoint = angle;
  }

  public float getAngleError() {
    return error;
  }

  public void doPID() {
    error = setpoint - (float) navX.getAngle();
    integral += (error * 0.2);
    derivative = (error - previous_error) / .02F;
    rcw = P * error + I * integral + D * derivative;
  }

  public boolean getLeftSensor() {
    return leftSensor.get();
  }

  public boolean getCenterSensor() {
    return centerSensor.get();
  }

  public boolean getRightSensor() {
    return rightSensor.get();
  }

  public void BrakeMode(IdleMode mode) {
    for (CANSparkMax motor : motors) {
      motor.setIdleMode(mode);
    }
  }

  public void setPID(double leftFront, double rightFront, double leftBack, double rightBack) {
    leftFrontPID.setReference(leftFront, ControlType.kVelocity);
    rightFrontPID.setReference(rightFront, ControlType.kVelocity);
    leftBackPID.setReference(leftBack, ControlType.kVelocity);
    rightBackPID.setReference(rightBack, ControlType.kVelocity);
  }

  public void setPIDConstants(double p, double i, double d, double f) {
    setP(p);
    setI(i);
    setD(d);
    setFF(f);
  }

  public void setP(double p) {
    leftFrontPID.setP(p);
    rightFrontPID.setP(p);
    leftBackPID.setP(p);
    rightBackPID.setP(p);
  }

  public void setI(double i) {
    leftFrontPID.setI(i);
    rightFrontPID.setI(i);
    leftBackPID.setI(i);
    rightBackPID.setI(i);

  }

  public void setD(double d) {
    leftFrontPID.setD(d);
    rightFrontPID.setD(d);
    leftBackPID.setD(d);
    rightBackPID.setD(d);
  }

  public void setFF(double f) {

    leftFrontPID.setFF(f);
    rightFrontPID.setFF(f);
    leftBackPID.setFF(f);
    rightBackPID.setFF(f);
  }
}
