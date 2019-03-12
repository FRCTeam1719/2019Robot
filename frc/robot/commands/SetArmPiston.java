/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmPiston extends Command {
  Arm arm;
  DoubleSolenoid.Value value;

  /**
   * Set the arm piston to an expicit value
   * @param _arm
   * @param _value
   */
  public SetArmPiston(Arm _arm, DoubleSolenoid.Value _value) {
    // Use requires() here to declare subsystem dependencies
    arm = _arm;
    value = _value;
    requires(arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    arm.setPiston(value);;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
