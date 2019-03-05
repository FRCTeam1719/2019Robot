package frc.robot.controllers;

import javax.swing.JToggleButton;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.controllers.Basic.DummyOperatorController;

public class ExtremeController extends DummyOperatorController {
    public ExtremeController(int port) {
        super(port);
    }

    public boolean getClimb() {
        return controller.getRawAxis(2) > 0.95;
    }

    @Override
    public JoystickButton raiseFront() {
        return new JoystickButton(controller, 9);
    }

    @Override
    public JoystickButton raiseBack() {
        return new JoystickButton(controller, 10);
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
    public JoystickButton lowerFront() {
        return new JoystickButton(controller, 7);
    }

    @Override
    public JoystickButton lowerBack() {
        return new JoystickButton(controller, 8);
    }

    @Override
    public JoystickButton climberDriveButton() {
        return new JoystickButton(controller, 11);
    }

    @Override
    public JoystickButton autoClimbButton() {
        return new JoystickButton(controller, 11);
    }
    @Override
    public JoystickButton armGoUp(){
        return new JoystickButton(controller, 3);
    }

    @Override
    public JoystickButton reZeroButton() {
        return new JoystickButton(controller, 8);
    }
    @Override
    public JoystickButton climbBackButton(){
        return new JoystickButton(controller, 12);
    }

    @Override
    public JoystickButton cameraSwitch() {
        return new JoystickButton(controller, 4);
    }

    @Override
    public double getY() {
        return controller.getY();
    }
}