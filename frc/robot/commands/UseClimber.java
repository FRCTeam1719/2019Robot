/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;

public class UseClimber extends Command {
  public enum ClimberOption {
    LOWER_BOTH, RAISE_FRONT, RAISE_BACK
  }

  Climber climber;
  ClimberOption option;

  public UseClimber(Climber _climber, ClimberOption _option) {
    super("Climber");
    requires(_climber);
    climber = _climber;
    option = _option;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (option == ClimberOption.LOWER_BOTH) {
      climber.lowerBoth();
    } else if (option == ClimberOption.RAISE_BACK) {
      climber.raiseBack();
    } else if (option == ClimberOption.RAISE_FRONT) {
      climber.raiseFront();
    } else {
      System.out.print("Oh no");
    }
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
