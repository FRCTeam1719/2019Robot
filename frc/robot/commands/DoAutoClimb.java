/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.UseClimber.ClimberOption;
import frc.robot.subsystems.Climber;

public class DoAutoClimb extends Command {
  Climber climber;
  float frontTiltRange = 2;
  float backTiltRange = 3;
  float TOLERANCE = 1.5F;
  float zero;

  public DoAutoClimb(Climber climber) {
    // Use requires() here to declare subsystem dependencies
    // requires(climber);

    this.climber = climber;
    frontTiltRange = 2;
    backTiltRange = 3;
    TOLERANCE = 1.5F;
  }

  int i;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.navX.reset();

    i = 1;
    // zero = Robot.navX.getPitch();  
    // climber.lowerBoth();
  }

  boolean frontOs = false;
  boolean backOs = true;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    float adjustedPitch = Robot.navX.getPitch() - zero;

    boolean frontTilted = (adjustedPitch > frontTiltRange) && !(adjustedPitch < -TOLERANCE);
    boolean backTilted = (adjustedPitch < -backTiltRange) && !(adjustedPitch > TOLERANCE);
    boolean closeEnough = Math.abs(adjustedPitch) < TOLERANCE;

    SmartDashboard.putBoolean("closeEnough", closeEnough);
    SmartDashboard.putBoolean("backTilted", backTilted);
    SmartDashboard.putBoolean("frontTilted", frontTilted);
    SmartDashboard.putNumber("Adjusted Pitch", adjustedPitch);
    SmartDashboard.putNumber("Back Tilt THold", backTiltRange);
    SmartDashboard.putNumber("Front Tilt THold", frontTiltRange);

    if (frontTilted) {
      climber.lowerFront();
      climber.offBack();
      if (!frontOs) {
        frontTiltRange *= 0.356;
        frontOs = true;
      }
    } else {
      if(frontOs == true){
        Timer.delay(.1);
      }
      frontOs = false;
    }

    if (backTilted) {
      climber.lowerBack();
      climber.offFront();
      if (!backOs) {
        backTiltRange *= 0.5;
        backOs = true;
      }
    } else {
      if(backOs == true){
        Timer.delay(.1);
      }
      backOs = false;
    }

    if (!frontTilted && !backTilted && closeEnough) {
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
