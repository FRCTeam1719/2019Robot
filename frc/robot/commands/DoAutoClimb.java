/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.UseClimber.ClimberOption;
import frc.robot.subsystems.Climber;

public class DoAutoClimb extends Command {
  Climber climber;
  float frontTiltRange = 5;
  float backTiltRange = 5;
  float TOLERANCE = 3;
  float zero;

  public DoAutoClimb(Climber climber) {
    // Use requires() here to declare subsystem dependencies
    // requires(climber);

    this.climber = climber;
  }
  int i;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.navX.reset();
    i = 1;
    zero = Robot.navX.getPitch();
    //climber.lowerBoth();
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean frontTilted = (Robot.navX.getPitch() + zero > frontTiltRange) && !(Robot.navX.getPitch() + zero > -TOLERANCE);
    boolean backTilted = (Robot.navX.getPitch() + zero < -backTiltRange) && !(Robot.navX.getPitch() + zero < TOLERANCE);
    boolean closeEnough = Math.abs(Robot.navX.getPitch() + zero) < TOLERANCE;

    SmartDashboard.putBoolean("closeEnough", closeEnough);
    SmartDashboard.putBoolean("backTilted", backTilted);
    SmartDashboard.putBoolean("frontTilted", frontTilted);
    SmartDashboard.putNumber("Pitch", Robot.navX.getPitch());
    i++;
    SmartDashboard.putNumber("Test", i);


    if (frontTilted) {
      climber.lowerFront();
      climber.offBack();
    } 
    
    if (backTilted) {
      climber.lowerBack();
      climber.offFront();
    } 

    if(!frontTilted && !backTilted && closeEnough){
      climber.lowerBoth();
    }

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
    Scheduler.getInstance().add(new UseClimber(Robot.climber, ClimberOption.OFF_BOTH));
  }
}
