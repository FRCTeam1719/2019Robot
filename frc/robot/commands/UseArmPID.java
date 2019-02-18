/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class UseArmPID extends Command {
  Arm arm;

  final double TOLERANCE = 1;
  final double CONTROL_SCALING = .75;
  final double LOW_RANGE_CONTROL_SCALING = .25;
  final double LOW_RANGE_THRESHOLD = -60;
  private double kP, kI, kD, kFF = 0;
  PIDController armPID;
  private double pidOut;

  private class ArmPIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			pidOut = output;
		}
    	
}


  public UseArmPID(Arm _arm) {
    arm = _arm;
    requires(arm);
    arm.armPot.setPIDSourceType(PIDSourceType.kDisplacement);
    armPID = new PIDController(kP, kI, kD, arm.armPot , new ArmPIDOutput());
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // _pidCont = arm.getPIDController();
    armPID.setInputRange(0D, 110D);
    armPID.setOutputRange(-1D, 1D);
    armPID.setSetpoint(arm.armPot.get());
    armPID.setPercentTolerance(TOLERANCE);
    armPID.enable();
    SmartDashboard.putData("Arm PID", armPID);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double joystickReading = Robot.oi.getOperatorY();
    double motorSpeed;

    SetPIDFromDashboard();

    arm.setMotor(Robot.oi.getOperatorY() / 7.5);

    if (Math.abs(joystickReading) < TOLERANCE) { // joystick not used, hold arm steady with PID + sinusoidally varing force
      armPID.enable();
      arm.setMotor(pidOut);
    } else { // joystick touched, reset integral and desired pos
      armPID.disable();
      armPID.setSetpoint(Robot.arm.getArmAngle());
      // Apply control scaling
      if (Robot.arm.getArmAngle() < LOW_RANGE_THRESHOLD)
        if (Math.abs(joystickReading) < 0) {
          motorSpeed = joystickReading * LOW_RANGE_CONTROL_SCALING;
        } else {
          motorSpeed = joystickReading * CONTROL_SCALING;
        }
      else {
        motorSpeed = joystickReading * CONTROL_SCALING;
      }
      if (motorSpeed > 0.7) {
        motorSpeed = 0.7;
      } else if (motorSpeed < -0.7) {
        motorSpeed = -0.7;
      }
      arm.setMotor(motorSpeed);
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
  }

  protected void SetPIDFromDashboard() {
    armPID = (PIDController)SmartDashboard.getData("Arm PID");
  }
}
