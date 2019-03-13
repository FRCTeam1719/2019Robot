/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vacuum;

public class UseReleaseValve extends Command {
  Vacuum vacuum;
  int BURST
  public UseReleaseValve(Vacuum vacuum) {
    requires(vacuum);
    this.vacuum = vacuum;
    }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    BURST = 500;
    vacuum.release();
    vacuum.set(0F);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if!(BURST <= 0){
      vacuum.release();
    }
    else {
      vacuum.suck();
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
    vacuum.suck();
    vacuum.set(.9F);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
