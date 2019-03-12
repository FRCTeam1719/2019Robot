/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwitchCamera extends Command {
  public enum SwitchMode {
    NEXT, PREV, TOGGLE
  }

  SwitchMode mode;
/**
 * Switch the camera please
 */
  public SwitchCamera() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    String incoming = (String) SmartDashboard.getString("Camera", "0");
    int tmp = Integer.parseInt(incoming);
    System.out.println("donba" + tmp);
    int outgoing = -1;
    if(tmp == 1){
      outgoing = 0;
    }else if (tmp == 0){
      outgoing = 1;
    }
    SmartDashboard.putString("Camera", Integer.toString(outgoing));
    System.out.println("donbe" + tmp);
    } 

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
