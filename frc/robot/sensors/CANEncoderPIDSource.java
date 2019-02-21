package frc.robot.sensors;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class CANEncoderPIDSource implements PIDSource {

    CANEncoder encoder;
    PIDSourceType source = PIDSourceType.kRate;
    public CANEncoderPIDSource(CANEncoder _encoder){
        encoder = _encoder;
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
            return encoder.getVelocity();
        } else if (source == PIDSourceType.kDisplacement){
            return encoder.getPosition();
        }
        return 0;
	}

}
