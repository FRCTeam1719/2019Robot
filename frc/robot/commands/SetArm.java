/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.subsystems.Arm;

/**
 * move arm to predesignated height
 */
public class SetArm extends Command {
  Arm arm;
  String location;
  int P, I, D = 1;
  double error, rcw, derivative;
  int integral, previous_error, setpoint = 0;
  Potentiometer pot;

  public SetArm(Arm _arm, String _location, Potentiometer _pot) {
    arm = _arm;
    pot = _pot;
    location = _location;
  }

  public void setSetpoint(int setpoint) {
    this.setpoint = setpoint;
  }

  public void PID() {
    error = setpoint - pot.get(); // Error = Target - Actual
    this.integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
                                    // IterativeRobot)
    derivative = (error - this.previous_error) / .02;
    this.rcw = P * error + I * this.integral + D * derivative;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    PID();
    arm.setMotor(rcw);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
