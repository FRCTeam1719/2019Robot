package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/*
This is an abstract class that models a preset for the controllers. 
*/
public abstract class DummyOperatorController {

    public  Joystick controller;

    public DummyOperatorController(int port) {
        controller = new Joystick(port);
    }

    public abstract boolean getClimb();

    public abstract JoystickButton raiseFront();

    public abstract JoystickButton raiseBack();

    public abstract JoystickButton releaseButton();

    public abstract JoystickButton toggleArm();

    public abstract JoystickButton lowerFront();

    public abstract JoystickButton lowerBack();

    public abstract JoystickButton climberDriveButton();

    public abstract JoystickButton autoClimbButton();

    public abstract JoystickButton armGoUp();

    public abstract JoystickButton reZeroButton();

    public abstract JoystickButton climbBackButton();
    public abstract JoystickButton cameraSwitch();
}


