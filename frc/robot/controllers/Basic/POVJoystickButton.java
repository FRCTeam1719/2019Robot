package frc.robot.controllers.Basic;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class POVJoystickButton extends Button {
    GenericHID joystick;
    int povNumber;
    double threshold;
    boolean invert;

    public POVJoystickButton(GenericHID _joystick, int _povNumber) {
        joystick = _joystick;
        povNumber = _povNumber;
    }
    public POVJoystickButton(GenericHID _joystick, int _povNumber, double _threshold) {
        this(_joystick,_povNumber);
    }

    @Override
    public boolean get() {
        return joystick.getPOV() == povNumber;
    }
}