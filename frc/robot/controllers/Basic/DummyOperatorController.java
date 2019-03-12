package frc.robot.controllers.Basic;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
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

    public abstract Button raiseFront();

    public abstract Button raiseBack();

    public abstract Button releaseButton();

    public abstract Button toggleArm();

    public abstract JoystickButton lowerFront();

    public abstract Button lowerBack();

    public abstract Button climberDriveButton();

    public abstract Button autoClimbButton();

    public abstract Button armGoUp();

    public abstract Button reZeroButton();

    public abstract Button climbBackButton();

    public abstract Button cameraSwitch();

    public abstract double getY();
}


