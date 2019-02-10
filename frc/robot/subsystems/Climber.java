/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  DoubleSolenoid frontSolenoid, backSolenoid;

  public Climber(DoubleSolenoid _frontSolenoid, DoubleSolenoid _backSolenoid) {
    frontSolenoid = _frontSolenoid;
    backSolenoid = _backSolenoid;

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void raiseBoth() {
    frontSolenoid.set(Value.kReverse);
    backSolenoid.set(Value.kReverse);
  }

  public void lowerBoth() {
    System.out.println("Lowering both. Oh no");
    frontSolenoid.set(Value.kForward);
    backSolenoid.set(Value.kForward);
  }

  public void raiseFront() {
    frontSolenoid.set(Value.kReverse);
  }

  public void raiseBack() {
    backSolenoid.set(Value.kReverse);
  }

  public void kill() {
    frontSolenoid.set(Value.kOff);
    backSolenoid.set(Value.kOff);
  }
}
