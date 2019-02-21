package frc.robot.controllers;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class XBoxController extends DummyDriverController {

    public XBoxController(int port){
        super(port);
    }
    
    @Override
    public JoystickButton getA() {
        return null;
    }

    @Override
    public JoystickButton getB() {
        return null;
    }

    @Override
    public JoystickButton getX() {
        return null;
    }

    @Override
    public JoystickButton getY() {
        return null;
    }

    @Override
    public double getLeftX() {
        return 0;
    }

    @Override
    public double getLeftY() {
        return 0;
    }

    @Override
    public double getRightY() {
        return 0;
    }

    @Override
    public double getRightX() {
        return 0;
    }

    @Override
    public double rightTrigger() {
        return 0;
    }

    @Override
    public double leftTrigger() {
        return 0;
    }

    @Override
    public JoystickButton rightBumper() {
        return null;
    }

    @Override
    public JoystickButton leftBumper() {
        return null;
    }

    @Override
    public int getDPAD() {
		return 0;
	}

}