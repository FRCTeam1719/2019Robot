/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  SpeedController highMotor;
  SpeedController lowMotor;

  public Intake(SpeedController _highMotor, SpeedController _lowMotor) {
    super("Intake");

    highMotor = _highMotor;
    lowMotor = _lowMotor;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void intake() {
    highMotor.set(.8);
    lowMotor.set(.8);
  }

  public void outake() {
    highMotor.set(-.8);
    lowMotor.set(-.8);
  }
  
  public void stop() {
    highMotor.stopMotor();
    lowMotor.stopMotor();
  }
}
