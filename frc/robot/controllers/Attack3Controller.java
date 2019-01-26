package frc.robot.controllers;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Attack3Controller extends DummyOperatorController {
    public Attack3Controller(int port) {
        super(port);
    }

    public boolean getClimb() {
        return controller.getRawAxis(2) > 0.95;
    }

    @Override
    public JoystickButton raiseFront() {
        return new JoystickButton(controller, 5);
    }

    @Override
    public JoystickButton raiseBack() {
        return new JoystickButton(controller, 6);
    }

    @Override
    public JoystickButton releaseButton() {
        return new JoystickButton(controller, 2);
    }
}