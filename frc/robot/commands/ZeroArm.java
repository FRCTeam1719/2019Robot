/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Arm;

public class ZeroArm extends Command {
  private Arm arm;
  private boolean done = false;
  public ZeroArm(Arm _arm) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    arm = _arm;
    requires(arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    arm.setMotor(.8);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(arm.GetUpperLimit()){
      arm.SetZero();
      done = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return done;
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
