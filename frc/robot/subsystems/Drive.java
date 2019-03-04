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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.UseDrive;
import frc.robot.commands.UseDriveCANPID;
import frc.robot.commands.UseDrivePID;
import frc.robot.commands.UseDriveRotPID;
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
  private boolean shift;

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

    SmartDashboard.putNumber("TEM", leftFrontPID.getI());

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
    robotDrive.driveCartesian(x, y, rot);
    // System.out.println("Dr" + gyro.getAngle());

    SmartDashboard.putNumber("Gyro", gyro.getAngle());

    SmartDashboard.putNumber("Gyro2", navX.getAngle());
    for (CANSparkMax motor : motors) {
      // System.out.println(motor.get());
    }
  }

  public void mecanumPolar(double magnitude, double angle, double rotation){
    robotDrive.drivePolar(magnitude, angle, rotation);
  }
  public void mecanumGyro(double x, double y, double rot, double gyroAngle) {
    robotDrive.driveCartesian(x, y, rot, gyroAngle);
    // System.out.println("Dr" + gyro.getAngle());
    for (CANSparkMax motor : motors) {
      // System.out.println(motor.get());
    }
  }

public boolean getShift(){
return shift;
}
public void setShift(boolean toShift){
  shift = toShift;
}

  public float getAngleError() {
    return error;
  }

  public void manual(double lf, double rf, double lb, double rb) {
    motors[0].set(lf);
    motors[1].set(rf);
    motors[2].set(lb);
    motors[3].set(rb);
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
  public void Stop(){
    for (CANSparkMax motor : motors){
      motor.set(0);
    }
  }
}
