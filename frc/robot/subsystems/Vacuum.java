/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.UseVacuum;

/**
 * Add your docs here.
 */
public class Vacuum extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private SpeedController motor;

    public Vacuum(SpeedController _motor) {
        motor = _motor;

    }

    @Override
    public void initDefaultCommand() {

    }

    /**
     * Set the speed of the motor for the vacuum. It will almost always be set to
     * the max speed of 1
     * 
     * @param speed - speed to spin the vacuum motor
     */
    public void set(double speed) {
        motor.set(speed);
    }

    /**
     * Reset the vacuum speed to 0
     */
    public void reset() {
        motor.set(0);
    }
}
