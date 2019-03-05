/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.controllers.Basic.DummyOperatorController;

/**
 * Add your docs here.
 */
public class WingmanController extends DummyOperatorController {
    public WingmanController(int port) {
        super(port);
    }

    @Override
    public boolean getClimb() {
        return false;
    }

    @Override
    public JoystickButton raiseFront() {
        return new JoystickButton(controller, 1);
    }

    @Override
    public JoystickButton raiseBack() {
        return new JoystickButton(controller, 2);
    }

    @Override
    public JoystickButton releaseButton() {
        return null;
    }

    @Override
    public JoystickButton toggleArm() {
        return null;
    }

    @Override
    public JoystickButton lowerFront() {
        return null;
    }

    @Override
    public JoystickButton lowerBack() {
        return null;
    }

    @Override
    public JoystickButton climberDriveButton() {
        return null;
    }

    @Override
    public JoystickButton autoClimbButton() {
        return null;
    }

    @Override
    public JoystickButton armGoUp() {
        return null;
    }

    @Override
    public JoystickButton reZeroButton() {
        return null;
    }

    @Override
    public JoystickButton climbBackButton() {
        return null;
    }

    @Override
    public JoystickButton cameraSwitch() {
		return null;
	}

    @Override
    public double getY() {
        return 0;
    }

}
