/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.UseDrive;



/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final int FRONT_LEFT = 0;
  private final int FRONT_RIGHT = 1;
  private final int BACK_LEFT = 2;
  private final int BACK_RIGHT = 3;

  private SpeedController[] motors;

  public Drive(SpeedController leftFrontMotor, SpeedController rightFrontMotor, SpeedController leftBackMotor,
      SpeedController rightBackMotor) {
    super();

    // 2D array of the speed controllers passed
    motors = new SpeedController[] { leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor };
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new UseDrive(this));
  }

  /**
   * Returns the motor speeds for a given mecanum-style input
   * 
   * @param x   the x position of the right stick
   * @param y   the y position of the right stick
   * @param rot the x position of the left stick
   * @return an array of motor speeds ([x][y])
   */
  public void mecanum(double x, double y, double rot) {
    // Iterate over each motor and assign it its respective motor speed
    motors[FRONT_LEFT].set(y + x + rot);
    motors[FRONT_RIGHT].set(y - x - rot);
    motors[BACK_LEFT].set(y - x + rot);
    motors[BACK_RIGHT].set(y + x - rot);

    // Calculate maximum
    double max = 0;
    for (SpeedController motor : motors) {
      if (Math.abs(motor.get()) > max) {
        max = Math.abs(motor.get());
      }
    }

    // Apply maximum
    for (SpeedController motor : motors) {
      motor.set(motor.get() / max);
    }
  }
}
