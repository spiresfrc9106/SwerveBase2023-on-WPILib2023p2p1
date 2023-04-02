package frc.hardwareWrappers.SwerveAzmthEncoder.RevEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.hardwareWrappers.SwerveAzmthEncoder.AbstractSwerveAzmthEncoder;
import frc.lib.Signal.Annotations.Signal;

public class RealRevThroughBoreEncoder extends AbstractSwerveAzmthEncoder {

    DigitalInput m_digitalInput;
    DutyCycle m_dutyCycle;

    //@Signal(units="Hz")
    double freq;

    public RealRevThroughBoreEncoder(int port){
        m_digitalInput = new DigitalInput(port);
        m_dutyCycle = new DutyCycle(m_digitalInput);
    }

    @Override
    public double getRawAngle_rad() {
        freq = m_dutyCycle.getFrequency(); //Track this for fault mode detection
       
        double dutyCycleAsRatio = m_dutyCycle.getOutput();
        double smallestDutyCycleSetToZeroRevolutions = dutyCycleAsRatio - 1.0/1025.0;
        // Per the https://docs.revrobotics.com/through-bore-encoder/specifications
        // and the https://docs.broadcom.com/doc/pub-005892
        // specifications:
        // A duty cycle of 1/1025 is 0  or 0/(65535+1) of a complete circle.
        // A duty cycle of 1024/1025 is 65535/(65535+1) of a complete circle.
        // The largest specified duty cycle is 1024/1025.
        // Having subtracted 1/1025, the largest specified duty cycle is then:
        // 1024/1025, which represents 65535/4096 of a circle.
        double revolutions = smallestDutyCycleSetToZeroRevolutions * (65535.0/65536.0) / (1024.0/1025.0);

        // noise may have let the result go below 0 or above 1.
        // we're not wrapping those back 0 to 1 or 0 to 2 pi because
        // the routines that call this use UnitUtils.wrapAngleRad.

        // Convert revolutions to radians
        double anglerad = - revolutions * 2 * Math.PI;

        return anglerad;
    }

    
}
