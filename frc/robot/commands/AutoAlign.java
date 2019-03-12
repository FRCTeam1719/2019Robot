/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;

public class AutoAlign extends Command {
  Drive drive;

  private boolean driverVision, tapeVision, cargoVision, cargoSeen, tapeSeen;
  private NetworkTableEntry tapeDetected, cargoDetected, tapeYaw, cargoYaw, videoTimestamp, driveWanted, tapeWanted,
      cargoWanted;
  private double kP = 0;
  private double targetAngle;

  NetworkTableInstance instance;
  NetworkTable chickenVision;
  /**
   * A "Very Simple" command for aligning with the output of chicken vision.
   * @param _drive
   */
  public AutoAlign(Drive _drive) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    drive = _drive;
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    chickenVision = instance.getTable("ChickenVision"); //Set up chicken vision varrables

    tapeDetected = chickenVision.getEntry("tapeDetected");
    cargoDetected = chickenVision.getEntry("cargoDetected");
    tapeYaw = chickenVision.getEntry("tapeYaw"); // The one we care about
    cargoYaw = chickenVision.getEntry("cargoYaw");

    driveWanted = chickenVision.getEntry("Driver");
    tapeWanted = chickenVision.getEntry("Tape");
    cargoWanted = chickenVision.getEntry("Cargo");

    videoTimestamp = chickenVision.getEntry("VideoTimestamp");

    driveWanted.setBoolean(false); //Tell the chicken vision to look for tape
    tapeWanted.setBoolean(true);
    cargoWanted.setBoolean(false);
    // Checks if vision sees cargo or vision targets. This may not get called unless
    // cargo vision detected
    tapeSeen = tapeDetected.getBoolean(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    UpdateTables();
    kP = SmartDashboard.getNumber("TTA Kp", kP); // Kp is the magic number
    double y = -Robot.oi.getDriverLeftY();

    y = y * Math.abs(y);

    if (tapeSeen) {
      targetAngle = tapeYaw.getDouble(0);
    } else {
      targetAngle = 0;
    }
    double output = limitOutput(-kP * targetAngle, 0.3);
    drive.mecanum(0, y, output);

  }
  private void UpdateTables(){
    tapeDetected = chickenVision.getEntry("tapeDetected");
    cargoDetected = chickenVision.getEntry("cargoDetected");
    tapeYaw = chickenVision.getEntry("tapeYaw");
    cargoYaw = chickenVision.getEntry("cargoYaw");

    driveWanted = chickenVision.getEntry("Driver");
    tapeWanted = chickenVision.getEntry("Tape");
    cargoWanted = chickenVision.getEntry("Cargo");

    videoTimestamp = chickenVision.getEntry("VideoTimestamp");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    interrupted();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    driveWanted.setBoolean(true);
    tapeWanted.setBoolean(false);
    cargoWanted.setBoolean(false);

    targetAngle = 0;
  }

  public double limitOutput(double number, double maxOutput) {
    if (number > 1.0) {
      number = 1.0;
    }
    if (number < -1.0) {
      number = -1.0;
    }

    if (number > maxOutput) {
      return maxOutput;
    }
    if (number < -maxOutput) {
      return -maxOutput;
    }

    return number;
  }
}
