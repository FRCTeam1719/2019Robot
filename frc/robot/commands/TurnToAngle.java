/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drive;

public class TurnToAngle extends Command {
  private static final float ANGLE_ERROR_THRESHOLD = 0.05f;
  Drive drive;
  float angle;

  public TurnToAngle(Drive drive, float angle) {
    // Use requires() here to declare subsystem dependencies
    requires(drive);

    this.drive = drive;
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drive.setAngleSetpoint(angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    drive.doPID();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return drive.getAngleError() < ANGLE_ERROR_THRESHOLD;
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
