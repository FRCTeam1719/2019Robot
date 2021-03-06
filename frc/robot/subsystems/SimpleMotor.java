/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.SimpleSpinMotor;

/**
 * Add your docs here.
 */
public class SimpleMotor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  SpeedController motor;
  double rampSpeed = 0;
  public SimpleMotor(SpeedController _motor) {
    super();
    motor = _motor;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new SimpleSpinMotor(this, 0));
  }

  public void spin(float speed) {
    
    if (motor.get() < speed){
      rampSpeed += speed - rampSpeed / 2;
    } if (speed - rampSpeed < .05){
      rampSpeed = speed;
    }
   motor.set(rampSpeed);
  }

  public void stop() {
    motor.stopMotor();
  }
}
