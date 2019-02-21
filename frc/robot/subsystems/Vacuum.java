/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.SimpleSpinMotor;
import frc.robot.commands.UseVacuum;

/**
 * Add your docs here.
 */
public class Vacuum extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  SpeedController motor;
  double rampSpeed = 0;
  Solenoid releaseValve;
  public Vacuum(SpeedController _motor, Solenoid _releaseValve) {
    super();
    motor = _motor;
    releaseValve = _releaseValve;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new UseVacuum(this, .9F));
  }
  public void release() {
    releaseValve.set(true);
  }

  public void suck() {
    releaseValve.set(false);
  }
  public void spin(float speed) {
    if (motor.get() < speed){
      rampSpeed += speed - rampSpeed / 2;
    } if (speed - rampSpeed < .05){
      rampSpeed = speed;
    }
    if (rampSpeed > speed){
      rampSpeed = speed;
    }
   motor.set(rampSpeed);
  }

  public void stop() {
    motor.stopMotor();
  }
  public boolean ValveState(){
    return releaseValve.get();
  }
}
