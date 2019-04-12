/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CompVisionSwitch extends Command {
  public enum SwitchMode {
    NEXT, PREV, TOGGLE
  }

  SwitchMode mode;
  NetworkTable chickenVision;
  NetworkTableEntry camera;
  NetworkTableInstance instance;


  public CompVisionSwitch() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    instance = NetworkTableInstance.getDefault();
 
        chickenVision = instance.getTable("ChickenVision");
        camera = chickenVision.getEntry("Camera");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //String incoming = (String) SmartDashboard.getString("Camera", "0");
    String incoming = camera.getString("2");
    int tmp = Integer.parseInt(incoming);
    System.out.println("donba" + tmp);
    int outgoing = -1;
    if(tmp == 1){
      outgoing = 2;
    }else if (tmp == 2){
      outgoing = 1;
    }else{
      outgoing = 1;
    }
    
    camera.setString(Integer.toString(outgoing));
    SmartDashboard.putString("donbe", Integer.toString(outgoing));
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
