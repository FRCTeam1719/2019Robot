package frc.robot.controllers.Driver;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.controllers.Basic.DummyOperatorController;

public class XBoxControllerBindings extends DummyOperatorController {

    public XBoxControllerBindings(int port){
        super(port);
    }

    @Override
    public boolean getClimb() {
        return false;
    }

    @Override
    public JoystickButton raiseFront() {
        return new JoystickButton(controller, 4);
    }

    @Override
    public JoystickButton raiseBack() {
        return new JoystickButton(controller, 3);
    }

    @Override
    public JoystickButton releaseButton() {
        return null;
    }

    @Override
    public JoystickButton toggleArm() {
        return new JoystickButton(controller, 7);
    }

    @Override
    public JoystickButton lowerFront() {
        return new JoystickButton(controller, 2);
    }

    @Override
    public JoystickButton lowerBack() {
        return new JoystickButton(controller, 1);
    }

    @Override
    public JoystickButton climberDriveButton() {
        return new JoystickButton(controller, 8);
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
        return new JoystickButton(controller, 9);
    }

    @Override
    public JoystickButton cameraSwitch() {
        return null;
    }

    @Override
    public double getY() {
        return controller.getY(Hand.kLeft);
    }
}