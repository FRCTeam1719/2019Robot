/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.BreakMode;
import frc.robot.commands.UseClimber;
import frc.robot.commands.UseVacuum;
import frc.robot.commands.UseClimber.ClimberOption;
import frc.robot.controllers.Attack3Controller;
import frc.robot.controllers.ChineseController;
import frc.robot.controllers.DummyDriverController;
import frc.robot.controllers.DummyOperatorController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  DummyDriverController driverJoystick = new ChineseController(0);
  DummyOperatorController operatorJoystick = new Attack3Controller(1);

  public double getDriverLeftX() {
    return driverJoystick.getLeftX();
  }

  public double getDriverLeftY() {
    return driverJoystick.getLeftY();
  }

  public double getDriverRightX() {
    return driverJoystick.getRightX();
  }

  public double getDriverRightY() {
    return driverJoystick.getRightY();
  }

  public JoystickButton breakButton;
  public JoystickButton coastButton;

  public JoystickButton releaseButton;


  public OI() {
    breakButton = driverJoystick.leftBumper();
    coastButton = driverJoystick.rightBumper();
    breakButton.whenPressed(new BreakMode(Robot.drive, IdleMode.kBrake));
    coastButton.whenPressed(new BreakMode(Robot.drive, IdleMode.kCoast));

    releaseButton = operatorJoystick.releaseButton();
    releaseButton.whenPressed(new UseVacuum(Robot.vacuum, 0));
    releaseButton.whenReleased(new UseVacuum(Robot.vacuum, .9F));

    if(operatorJoystick.getClimb()){
      operatorJoystick.raiseBack().whenPressed(new UseClimber(Robot.climber, ClimberOption.RAISE_BACK));
      operatorJoystick.raiseFront().whenPressed(new UseClimber(Robot.climber, ClimberOption.RAISE_FRONT));
    }
  }
}