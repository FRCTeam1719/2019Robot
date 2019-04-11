
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
import frc.robot.commands.ClimberDrive;
import frc.robot.commands.DoAutoClimb;
import frc.robot.commands.SetPhotonCannon;
import frc.robot.commands.SwitchCamera;
import frc.robot.commands.ToggleArm;
import frc.robot.commands.TrackTarget;
import frc.robot.commands.UseClimber;
import frc.robot.commands.UseDriveRotPID;
import frc.robot.commands.UseFakeShift;
import frc.robot.commands.UseReleaseValve;
import frc.robot.commands.UseVacuum;
import frc.robot.commands.UseClimber.ClimberOption;
import frc.robot.controllers.Attack3Controller;
import frc.robot.controllers.ChineseController;
import frc.robot.controllers.XBoxControllerBindings;
import frc.robot.controllers.DummyDriverController;
import frc.robot.controllers.DummyOperatorController;
import frc.robot.controllers.FinalController;

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
  DummyOperatorController operatorJoystick = new XBoxControllerBindings(1);

  private int dpad;

  public boolean getDriverLeftBumper() {
    return driverJoystick.leftBumper().get();
  }

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

  public double getOperatorY() {
    return operatorJoystick.getY();
  }

  public JoystickButton breakButton;
  public JoystickButton coastButton;

  public JoystickButton releaseButton;

  public OI() {
    // breakButton = driverJoystick.leftBumper();
    // coastButton = driverJoystick.rightBumper();
    // breakButton.whenPressed(new BreakMode(Robot.drive, IdleMode.kBrake));
    // coastButton.whenPressed(new BreakMode(Robot.drive, IdleMode.kCoast));

    // releaseButton = operatorJoystick.releaseButton();
    // releaseButton.whenPressed(new UseVacuum(Robot.vacuum, 0));
    // releaseButton.whenPressed(new UseReleaseValve(Robot.arm, true));
    // releaseButton.whenReleased(new UseVacuum(Robot.vacuum, .9F));
    // releaseButton.whenReleased(new UseReleaseValve(Robot.arm, false));

    // JoystickButton lower1Button = operatorJoystick.lower1Button();
    // lower1Button.whenPressed(new UseClimber(Robot.climber,
    // ClimberOption.LOWER_FRONT));
    // lower1Button.whenReleased(new UseClimber(Robot.climber,
    // ClimberOption.OFF_FRONT));

    // JoystickButton lower2Button = operatorJoystick.lower2Button();
    // lower2Button.whenPressed(new UseClimber(Robot.climber,
    // ClimberOption.LOWER_BACK));
    // lower2Button.whenReleased(new UseClimber(Robot.climber,
    // ClimberOption.OFF_BACK));

    //JoystickButton autoClimb = operatorJoystick.autoClimbButton();
    //autoClimb.whileHeld(new DoAutoClimb(Robot.climber));
    // autoClimb.whenPressed(new UseClimber(Robot.climber,
    // ClimberOption.LOWER_BOTH));

    JoystickButton raiseFront = operatorJoystick.lowerFront();
    raiseFront.whenPressed(new UseClimber(Robot.climber, ClimberOption.LOWER_FRONT));
    raiseFront.whenReleased(new UseClimber(Robot.climber, ClimberOption.OFF_FRONT));
   
    JoystickButton raiseBack = operatorJoystick.lowerBack();
    raiseBack.whenPressed(new UseClimber(Robot.climber, ClimberOption.LOWER_BACK));
    raiseBack.whenReleased(new UseClimber(Robot.climber, ClimberOption.OFF_BACK));

    JoystickButton toggleArmButton = operatorJoystick.toggleArm();
    // toggleArmButton.whenPressed(new ToggleArm(Robot.arm));
    toggleArmButton.whileHeld(new UseReleaseValve(Robot.vacuum));
    JoystickButton climbDrive = operatorJoystick.climberDriveButton();
    climbDrive.whenPressed(new ClimberDrive(Robot.climber, .45));
    climbDrive.whenReleased(new ClimberDrive(Robot.climber, 0));

    JoystickButton armGoUp = operatorJoystick.armGoUp();
    armGoUp.whenPressed(new ToggleArm(Robot.arm));

    JoystickButton upBack = operatorJoystick.raiseBack();
    upBack.whenPressed(new UseClimber(Robot.climber, ClimberOption.RAISE_BACK));
    upBack.whenReleased(new UseClimber(Robot.climber, ClimberOption.OFF_BACK));

    JoystickButton upFront = operatorJoystick.raiseFront();
    upFront.whenPressed(new UseClimber(Robot.climber, ClimberOption.RAISE_FRONT));
    upFront.whenReleased(new UseClimber(Robot.climber, ClimberOption.OFF_FRONT));

    JoystickButton backClimb = operatorJoystick.climbBackButton();
    backClimb.whenPressed(new ClimberDrive(Robot.climber, -.35));
    backClimb.whenReleased(new ClimberDrive(Robot.climber, 0));


    JoystickButton shiftDown = driverJoystick.leftBumper();
    JoystickButton shiftUp = driverJoystick.rightBumper();

    shiftDown.whenPressed(new UseFakeShift(Robot.drive, true));
    shiftUp.whenPressed(new UseFakeShift(Robot.drive, false));


    JoystickButton cameraButton = driverJoystick.cameraSwitch();
    cameraButton.whenPressed(new SwitchCamera());

    JoystickButton lightButton = driverJoystick.lightButton();
    lightButton.whenPressed(new SetPhotonCannon(Robot.photonCannon, true));
    lightButton.whenReleased(new SetPhotonCannon(Robot.photonCannon, false));
    lightButton.whileHeld(new TrackTarget(Robot.drive));
    // Turn to angle
    dpad = driverJoystick.getDPAD();
    double angleUnit = 45; // 360/8
    double angleToTurn = angleUnit * dpad;

  }
}