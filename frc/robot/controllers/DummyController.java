package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/*
This is an abstract class that models a preset for the controllers. 
*/
public abstract class DummyController {

    protected Joystick controller;

    public DummyController(int port) {
        controller = new Joystick(port);
    }

    protected abstract JoystickButton getA();

    protected abstract JoystickButton getB();

    protected abstract JoystickButton getX();

    protected abstract JoystickButton getY();

    protected abstract double getLeftX();

    protected abstract double getLeftY();

    protected abstract double getRightY();

    protected abstract double getRightX();

    protected abstract double rightTrigger();

    protected abstract double leftTrigger();

    protected abstract JoystickButton rightBumper();

    protected abstract JoystickButton leftBumper();

}
