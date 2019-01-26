/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Arm subsystem. The arm can hold balls and hatches, and has the ability to
 * extend the arm upwards using a motor and move the entire mechanism up using a
 * piston
 */
public class Arm extends Subsystem {

  private SpeedController motor;

  private Solenoid piston;

  public Arm(SpeedController _motor, Solenoid _piston) {
    motor = _motor;
    piston = _piston;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setMotor(double speed) {
    motor.set(speed);
  }

  /**
   * Put the arm up to reach the higher levels
   */
  public void putUp() {
    piston.set(true);
  }

  /**
   * Put the arm back down to reach lower levels
   */
  public void putDown() {
    piston.set(false);
  }

  /**
   * toggles the piston
   */
  public void toggle() {
    piston.set(!getState());
  }

  public boolean getState() {
    return piston.get();
  }
}
