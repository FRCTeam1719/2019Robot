/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

public static CANSparkMax leftFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
  public static CANSparkMax rightFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
  public static CANSparkMax leftBackMotor = new CANSparkMax(3, MotorType.kBrushless);
  public static CANSparkMax rightBackMotor = new CANSparkMax(4, MotorType.kBrushless);
  
  public static SpeedController vacuum = new Spark(3);

  public static CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  public static DoubleSolenoid armSolenoid = new DoubleSolenoid(0, 1);

  public static AHRS navX = new AHRS(I2C.Port.kMXP);
  public static AnalogGyro gyro = new AnalogGyro(0);

  public static DoubleSolenoid frontPiston = new DoubleSolenoid(2, 3);
  public static DoubleSolenoid backPiston = new DoubleSolenoid(4, 5);
  public static SpeedController climbDrive = new Spark(2);

  public static Compressor compressor = new Compressor(0);

  public static DigitalInput leftSensor = new DigitalInput(2);
  public static DigitalInput centerSensor = new DigitalInput(3);
  public static DigitalInput rightSensor = new DigitalInput(4);

  public static DigitalInput upperArmLimit = new DigitalInput(6);
  public static DigitalInput lowerArmLimit = new DigitalInput(5);
  public static Solenoid releaseValve = new Solenoid(7);

  public static DigitalInput frontTilt = new DigitalInput(8);
  public static DigitalInput backTilt = new DigitalInput(7);

public static Potentiometer armPot = new AnalogPotentiometer(1);

public static Ultrasonic ultrasonic = new Ultrasonic(null, null);
} 