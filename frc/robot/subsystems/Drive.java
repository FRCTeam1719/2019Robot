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
import com.revrobotics.CANSparkMax;
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
    robotDrive.driveCartesian(-x, y, rot, (gyro.getAngle() % 360) - 180);
    System.out.println("Dr" + gyro.getAngle());
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
}
