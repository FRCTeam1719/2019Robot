package frc.robot.controllers.Operator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.controllers.Basic.DummyOperatorController;

public class FinalController extends DummyOperatorController{

    public FinalController(int port){
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
        return new JoystickButton(controller, 1);
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
        return new JoystickButton(controller, 3);
    }

    @Override
    public JoystickButton lowerBack() {
        return new JoystickButton(controller, 2);
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
        return new JoystickButton(controller, 5);
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