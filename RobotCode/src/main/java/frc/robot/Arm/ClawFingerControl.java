package frc.robot.Arm;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;

public class ClawFingerControl {
    WrapperedCANMotorCtrl fingerMotorCtrl;
    double desState_speed_radpersec;
    double prvState_speed_radpersec = Double.NaN;
    double actState_speed_radpersec;
    int desCurrentLimit_A;
    int prvCurrentLimit_A = 0;
    double motorDesSpd_radpersec;
    SimpleMotorFeedforward fingerMotorFF;
    public double smoothCurrentAmps = 1e10;
    public double latestCurrentAmps;

    public ClawFingerControl(String fingerName, int fingerMotorIdx, boolean invertFinger) {
        prvCurrentLimit_A = 20;
        fingerMotorCtrl = new WrapperedCANMotorCtrl("finger" + fingerName, fingerMotorIdx, WrapperedCANMotorCtrl.CANMotorCtrlType.SPARK_MAX, prvCurrentLimit_A);
        fingerMotorCtrl.setInverted(invertFinger);
        fingerMotorFF = new SimpleMotorFeedforward(0, 0);
    }

    public void update() {
        if (Double.isNaN(prvState_speed_radpersec) || (prvState_speed_radpersec != desState_speed_radpersec)) {
            fingerMotorCtrl.setClosedLoopCmd(desState_speed_radpersec, fingerMotorFF.calculate(desState_speed_radpersec));
            prvState_speed_radpersec = desState_speed_radpersec;
        }
        if (prvCurrentLimit_A != desCurrentLimit_A) {
            fingerMotorCtrl.setCurrent_A(desCurrentLimit_A);
            prvCurrentLimit_A = desCurrentLimit_A;
        }
        fingerMotorCtrl.update();
        actState_speed_radpersec = fingerMotorCtrl.getVelocity_radpersec();
        double currentA = fingerMotorCtrl.getCurrent_A();
        if (smoothCurrentAmps > 1e9) {
            smoothCurrentAmps = currentA;
        } else {
            smoothCurrentAmps = smoothCurrentAmps * 0.8 + currentA * 0.2;
        }
        latestCurrentAmps = currentA;
    }

    public void updateTelemetry() {}

    public void setDesiredState(double speed_radpersec, int currentLimit_A) {
        desState_speed_radpersec = speed_radpersec;
        desCurrentLimit_A = currentLimit_A;
    }

    public void setClosedLoopGains(double wheel_kP, double wheel_kI, double wheel_kD, double wheel_kV, double wheel_kS) {
        fingerMotorCtrl.setClosedLoopGains(wheel_kP, wheel_kI, wheel_kD);
        fingerMotorFF = new SimpleMotorFeedforward(wheel_kS, wheel_kV);
    }

    public void resetWheelEncoder() {
        fingerMotorCtrl.resetDistance();
    }
}