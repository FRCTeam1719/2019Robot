package frc.robot.controllers;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This is a preset for the chinese knock-off playstation controller, and all of
 * its buttons.
 * 
 */
public class ChineseController extends DummyController {

    public ChineseController(int port) {
        super(port);
    }

    public double getLeftX() {
        return controller.getRawAxis(0);
    }

    protected double getLeftY() {
        return controller.getRawAxis(1);
    }

    protected double getRightY() {
        return controller.getRawAxis(5);
    }

    protected double getRightX() {
        return controller.getRawAxis(4);
    }

    @Override
    protected JoystickButton getA() {
        return new JoystickButton(controller, 0);
    }

    @Override
    // TODO: find values for the buttons once the wiki is up
    protected JoystickButton getB() {
        return new JoystickButton(controller, 1);
    }

    @Override
    protected JoystickButton getX() {
        return new JoystickButton(controller, 1);
    }

    @Override
    protected JoystickButton getY() {
        return new JoystickButton(controller, 1);
    }

    @Override
    protected double rightTrigger() {
        return 0;
    }

    @Override
    protected double leftTrigger() {
        return 0;
    }

    @Override
    protected JoystickButton rightBumper() {
        return new JoystickButton(controller, 1);
    }

    @Override
    protected JoystickButton leftBumper() {
        return new JoystickButton(controller, 1);
    }

}
