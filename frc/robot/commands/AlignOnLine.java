/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drive;

public class AlignOnLine extends Command {
  Drive drive;
  double rotSpeed;


  double ySpeed = 0.5;

  double correction = 0.3;

  double pDistance = 0.15;

  Ultrasonic ultrasonic;
/**
 * A "Simple" Command to allign a robot with the alignment line.
 * @param _drive
 * @param _ultrasonic
 */
  public AlignOnLine(Drive _drive, Ultrasonic _ultrasonic) {
    // Use requires() here to declare subsystem dependencies
    requires(_drive);
    ultrasonic = _ultrasonic;
    drive = _drive;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rotSpeed = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!drive.getRightSensor() && !drive.getLeftSensor()){
    
    if(drive.getCenterSensor()){
      rotSpeed = 0;
    }
    if(drive.getLeftSensor()){
      rotSpeed = -correction;
    }
    else if(drive.getRightSensor()){
      rotSpeed = correction;
    }
    else{
      rotSpeed = 0;
    }
    ySpeed = ultrasonic.getRangeInches() * pDistance; //P Loop!
    
    drive.mecanum(ySpeed, 0, rotSpeed);
  }
  else{
    rotSpeed = 0;
    ySpeed = 0;
    drive.Stop();
  }
}

  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; //Please use this is a while held
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
