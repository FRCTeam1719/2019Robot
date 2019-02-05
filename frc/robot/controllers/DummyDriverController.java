package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/*
This is an abstract class that models a preset for the controllers. 
*/
public abstract class DummyDriverController {

    public Joystick controller;

    public DummyDriverController(int port) {
        controller = new Joystick(port);
    }

    public abstract JoystickButton getA();

    public abstract JoystickButton getB();

    public abstract JoystickButton getX();

    public abstract JoystickButton getY();

    public abstract double getLeftX();

    public abstract double getLeftY();

    public abstract double getRightY();

    public abstract double getRightX();

    public abstract double rightTrigger();

    public abstract double leftTrigger();

    public abstract JoystickButton rightBumper();

    public abstract JoystickButton leftBumper();

    public abstract int getDPAD();

}
