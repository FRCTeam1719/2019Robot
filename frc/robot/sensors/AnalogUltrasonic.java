package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AnalogUltrasonic implements PIDSource{
public AnalogInput ultrasonic;
private double CONVERT = 0;

PIDSourceType source = PIDSourceType.kDisplacement;

    /**
     * A wrapper for an ultrasonic that scales the voltage to CM.
     */
    public AnalogUltrasonic(AnalogInput _ultrasonic){
        ultrasonic = _ultrasonic;
        ultrasonic.setAverageBits(4);
    }

    public double getRaw() {
        return ultrasonic.getVoltage();
    }
    /**
     * multiply the raw by a found constant
     */
    public double getCM() {
        return ultasonic.get * 255.608659214108; //This is just the slope that fits mesured distances well don't worry about it.
    }
    
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return null;
    }

    @Override
    public double pidGet() {
        return 0;
	}

}