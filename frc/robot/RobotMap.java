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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;

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

  public static CANSparkMax leftFrontMotor = new CANSparkMax(0, MotorType.kBrushless);
  public static CANSparkMax rightFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
  public static CANSparkMax leftBackMotor = new CANSparkMax(1, MotorType.kBrushless);
  public static CANSparkMax rightBackMotor = new CANSparkMax(3, MotorType.kBrushless);

  public static SpeedController vacuum = new Spark(4);

  public static AHRS navX = new AHRS(I2C.Port.kMXP);

  public static Solenoid frontPiston = new Solenoid(0);
  public static Solenoid backPiston = new Solenoid(1);

  
  public static DigitalInput leftSensor = new DigitalInput(0);
  public static DigitalInput centerSensor = new DigitalInput(1);
  public static DigitalInput rightSensor = new DigitalInput(2);
}