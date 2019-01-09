package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A Hatch device that has three pistons (sol1,sol2,sol3). Plan to give it better names once the device is
 * an actual device. 
 */

public class Hatch extends Subsystem {

   private Solenoid sol1, sol2, sol3;

    public Hatch(Solenoid s1, Solenoid s2, Solenoid s3){
        sol1 = s1;
        sol2 = s2;
        sol3 = s3;
    }

    public void release(){
        sol1.set(true);
        sol2.set(true);
        sol3.set(true);
    }

    public void reset(){
        sol1.close();
        sol2.close();
        sol3.close();
    }



    @Override
    protected void initDefaultCommand() {

    }

}
