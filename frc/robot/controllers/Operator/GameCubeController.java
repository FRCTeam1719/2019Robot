package frc.robot.controllers.Operator;

import javax.swing.JToggleButton;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.controllers.Basic.DummyOperatorController;
import frc.robot.controllers.Basic.POVJoystickButton;

public class GameCubeController extends DummyOperatorController {
    public GameCubeController(int port) {
        super(port);
    }

    public boolean getClimb() {
        return false;
    }

    @Override
    public JoystickButton raiseFront() {
        return new JoystickButton(controller, 13);
    }

    @Override
    public JoystickButton raiseBack() {
        return new JoystickButton(controller, 14);
    }

    @Override
    public JoystickButton releaseButton() {
        return toggleArm();
    }

    @Override
    public JoystickButton toggleArm() {
        return new JoystickButton(controller, 1);
    }
    
    @Override
    public JoystickButton lowerFront() {
        return new JoystickButton(controller, 10);
    }

    @Override
    public JoystickButton lowerBack() {
        return new JoystickButton(controller, 9);
    }

    @Override
    public POVJoystickButton climberDriveButton() {
        return new POVJoystickButton(controller, 1);
    }

    @Override
    public JoystickButton autoClimbButton() {
        return new JoystickButton(controller, 2);
    }
    @Override
    public JoystickButton armGoUp(){
        return new JoystickButton(controller, 1);
    }

    @Override
    public JoystickButton reZeroButton() {
        return new JoystickButton(controller, 8);
    }
    @Override
    public POVJoystickButton climbBackButton(){
        return new POVJoystickButton(controller, 5);
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