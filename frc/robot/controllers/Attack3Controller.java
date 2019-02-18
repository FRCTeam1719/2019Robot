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

    @Override
    public JoystickButton toggleArm() {
        return new JoystickButton(controller, 1);
    }
    
    @Override
    public JoystickButton lower1Button() {
        return new JoystickButton(controller, 10);
    }

    @Override
    public JoystickButton lower2Button() {
        return new JoystickButton(controller, 9);
    }

    @Override
    public JoystickButton climberDriveButton() {
        return new JoystickButton(controller, 7);
    }

    @Override
    public JoystickButton autoClimbButton() {
        return new JoystickButton(controller, 11);
    }
    @Override
    public JoystickButton armGoUp(){
        return new JoystickButton(controller, 3);
    }
}