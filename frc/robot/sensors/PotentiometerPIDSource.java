package frc.robot.sensors;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

public class PotentiometerPIDSource implements PIDSource {

    Potentiometer pot;
    PIDSourceType source = PIDSourceType.kDisplacement;
    public PotentiometerPIDSource(Potentiometer _pot){
        pot = _pot;
    }
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        source = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return source;
    }

    @Override
    public double pidGet() {
        if(source == PIDSourceType.kRate){
            return pot.get();
        } else if (source == PIDSourceType.kDisplacement){
            return pot.get();
        }
        return 0;
	}

}
