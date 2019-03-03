/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwitchCamera;
import frc.robot.commands.UseClimber;
import frc.robot.commands.UseVacuum;
import frc.robot.commands.SwitchCamera.SwitchMode;
import frc.robot.commands.UseClimber.ClimberOption;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vacuum;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Drive drive;
  public static Climber climber;
  public static Vacuum vacuum;
  public static Arm arm;
  public static AHRS navX;

  public static OI oi;

  public enum RobotMode {
    DRIVING, CLIMBING
  }

  public boolean lastClimb;
  public RobotMode state;
  Command autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    navX = RobotMap.navX;
    drive = new Drive(RobotMap.leftFrontMotor, RobotMap.rightFrontMotor, RobotMap.leftBackMotor,
        RobotMap.rightBackMotor, RobotMap.navX, RobotMap.leftSensor, RobotMap.centerSensor, RobotMap.rightSensor, RobotMap.gyro);
    climber = new Climber(RobotMap.frontPiston, RobotMap.backPiston, RobotMap.climbDrive, RobotMap.frontTilt, RobotMap.backTilt);
    state = RobotMode.DRIVING;
    vacuum = new Vacuum(RobotMap.vacuum, RobotMap.releaseValve);
    arm = new Arm(RobotMap.arm, RobotMap.armSolenoid, RobotMap.lowerArmLimit, RobotMap.upperArmLimit, RobotMap.armPot);

    oi = new OI();
    drive.BrakeMode(IdleMode.kBrake);
    lastClimb = false;
    SmartDashboard.putString("Camera", "0");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // System.out.println(RobotMap.frontTilt.get());
    // System.out.println(RobotMap.backTilt.get());
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    climber.kill();
    zero = 0F;    
  }

  @Override
  public void disabledPeriodic() {
    float frontTiltRange = 2;
    float backTiltRange = 3;
    float TOLERANCE = 1.5F;
    float adjustedPitch = Robot.navX.getPitch() - zero;

  if (zero == 0) zero = Robot.navX.getPitch();

  boolean frontTilted = (adjustedPitch > frontTiltRange) && !(adjustedPitch < -TOLERANCE);
  boolean backTilted = (adjustedPitch < -backTiltRange) && !(adjustedPitch > TOLERANCE);
  boolean closeEnough = Math.abs(adjustedPitch) < TOLERANCE;

  SmartDashboard.putBoolean("closeEnough", closeEnough);
  SmartDashboard.putBoolean("backTilted", backTilted);
  SmartDashboard.putBoolean("frontTilted", frontTilted);
  SmartDashboard.putNumber("Adjusted Pitch", adjustedPitch);

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
teleopInit();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }
float zero;
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    zero = Robot.navX.getPitch();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

     Scheduler.getInstance().add(
       new UseClimber(climber, ClimberOption.RAISE_BOTH)
    );

    RobotMap.compressor.setClosedLoopControl(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  boolean cameraOnce = false;
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    float frontTiltRange = 2;
    float backTiltRange = 3;
    float TOLERANCE = .5F;

    boolean frontTilt = (Robot.navX.getPitch() + zero > frontTiltRange);
    boolean backTilt = (Robot.navX.getPitch() + zero < -backTiltRange);
    boolean closeEnough = Math.abs(Robot.navX.getPitch() + zero) < TOLERANCE;

    SmartDashboard.putBoolean("closeEnough", closeEnough);
    SmartDashboard.putBoolean("backTilt", backTilt);
    SmartDashboard.putBoolean("frontTilt", frontTilt);
    SmartDashboard.putNumber("Pitch", Robot.navX.getPitch());
    SmartDashboard.putBoolean("Compressing", RobotMap.compressor.enabled());

    /*float frontTiltRange = 5;
    float backTiltRange = 7;
    float TOLERANCE = 4;
    SmartDashboard.putNumber("Pitch", navX.getPitch());
    boolean frontTilt = (Robot.navX.getPitch() > frontTiltRange);
    boolean backTilt = (Robot.navX.getPitch() < -backTiltRange);
    boolean closeEnough = Math.abs(Robot.navX.getPitch()) < TOLERANCE;

    SmartDashboard.putBoolean("closeEnough", closeEnough);
    SmartDashboard.putBoolean("backTilt", backTilt);
    SmartDashboard.putBoolean("frontTilt", frontTilt);
    SmartDashboard.putNumber("Pitch", Robot.navX.getPitch());*/
    // System.out.println("Robot " + RobotMap.navX.getAngle());
    // if (oi.operatorJoystick.getClimb() && !lastClimb) {
    //   climber.lowerBoth();
    // }
    // lastClimb = oi.operatorJoystick.getClimb();

    //System.out.println("DPAD: " + oi.driverJoystick.getDPAD());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}