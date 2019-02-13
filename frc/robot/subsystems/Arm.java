/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.RobotMap;
import frc.robot.commands.UseArm;

/**
 * Arm subsystem. The arm can hold balls and hatches, and has the ability to
 * extend the arm upwards using a motor and move the entire mechanism up using a
 * piston
 */
public class Arm extends PIDSubsystem {

  private SpeedController motor;
  private DoubleSolenoid piston;

  DigitalInput upperLimitSwitch;
  DigitalInput lowerLimitSwitch;
  
  Potentiometer pot;

  public Arm(SpeedController _motor, DoubleSolenoid _piston, DigitalInput _upperLimitSwitch, DigitalInput _lowerLimitSwitch) {
    super("Arm", 2.0, 0, 0);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);
    motor = _motor;
    piston = _piston;
    upperLimitSwitch = _upperLimitSwitch;
    lowerLimitSwitch = _lowerLimitSwitch;

    setDefaultCommand(new UseArm(this));
  }

  public void setMotor(double speed) {
    if (upperLimitSwitch.get())
      speed = Math.max(speed, 0);
    // else if (lowerLimitSwitch.get())
    //   speed = Math.min(speed, 0);

    motor.set(speed);
  }

  /**
   * Put the arm up to reach the higher levels
   */
  public void putUp() {
    piston.set(Value.kForward);
  }

  /**
   * Put the arm back down to reach lower levels
   */
  public void putDown() {
    piston.set(Value.kReverse);
  }

  public void toggle() {
    if (piston.get() == Value.kReverse) {
      putUp();
    } else if (piston.get() == Value.kForward) {
      putDown();
    }
  }

  public Value getState() {
    return piston.get();
  }

  @Override
  protected double returnPIDInput() {
    return pot.get();
  }

  @Override
  protected void usePIDOutput(double output) {
    motor.pidWrite(output);
  }

  @Override
  protected void initDefaultCommand() {
    
  }
}
