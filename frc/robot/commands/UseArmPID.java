/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class UseArmPID extends Command {
  Arm arm;
  
	final double TOLERANCE = 0.1;
	final double CONTROL_SCALING = .75;
	final double LOW_RANGE_CONTROL_SCALING = .25;
	final double LOW_RANGE_THRESHOLD = -60;
	private double lastErr = 0.0D;
private double integral = 0.0D;

  public UseArmPID(Arm _arm) {
    arm = _arm;
    requires(arm);
  }
  CANPIDController _pidCont;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _pidCont = arm.getPIDController();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double joystickReading = Robot.oi.getOperatorY();
    double motorSpeed;
    arm.setMotor(Robot.oi.getOperatorY() / 7.5);

    if(Math.abs(joystickReading) < TOLERANCE){ // joystick not used, hold arm steady with PID + sinusoidally varing force
      double kP = SmartDashboard.getNumber("Arm steady kP" , 0);
      double kI = SmartDashboard.getNumber("Arm steady kI", 0);
      double kD = SmartDashboard.getNumber("Arm steady kD", 0);
        double rng = SmartDashboard.getNumber("Arm steady integral range", 0);
        double angle = Robot.arm.getArmAngle();
        double error = -(Robot.arm.getTargetPos() - angle);
        if(Math.abs(error) < rng) integral += error;
        double derivative = error - lastErr;
        
        motorSpeed = kP * error + kI * integral + kD * derivative;
    }
        else { // joystick touched, reset integral and desired pos
          integral = 0;
          Robot.arm.setTargetPos(Robot.arm.getArmAngle());
        //Apply control scaling
          if(Robot.arm.getArmAngle()<LOW_RANGE_THRESHOLD)
            if (joystickReading < 0) {
              motorSpeed = joystickReading *LOW_RANGE_CONTROL_SCALING;
            }
            else {
              motorSpeed = joystickReading * CONTROL_SCALING;
            }
          else
            motorSpeed = joystickReading * CONTROL_SCALING;
      }
      if (motorSpeed > 0.7) {
        motorSpeed = 0.7;
      }
      else if (motorSpeed < -0.7) {
        motorSpeed = -0.7;
  }
  arm.setMotor(motorSpeed);
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
  }
}
