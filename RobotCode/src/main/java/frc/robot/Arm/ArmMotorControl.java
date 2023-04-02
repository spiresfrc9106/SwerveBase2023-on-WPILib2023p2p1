package frc.robot.Arm;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.Robot;

public class ArmMotorControl {

    WrapperedCANMotorCtrl armMotorCtrl;

    double desState_speed_radpersec;
    double prvState_speed_radpersec = Double.NaN;
    double actState_speed_radpersec;
    int prvCurrentLimit_A=0;


    int desCurrentLimit_A;

    double motorDesSpd_radpersec;

    //frc.lib.Signal.Signal fingSpdDesSig;
    //frc.lib.Signal.Signal fingSpdActSig;
    //frc.lib.Signal.Signal fingCurActSig;

    SimpleMotorFeedforward armMotorFF;

    //@Signal (units="double")
    public double smoothCurrentAmps = 1e10;
    //@Signal (units="double")
    public double latestCurrentAmps;

    public ArmMotorControl(String armName, int armMotorIdx, boolean invertArm){
        prvCurrentLimit_A = 40;
        armMotorCtrl = new WrapperedCANMotorCtrl("arm"+armName, armMotorIdx, WrapperedCANMotorCtrl.CANMotorCtrlType.SPARK_MAX, prvCurrentLimit_A);
      
        armMotorCtrl.setInverted(invertArm);

        armMotorFF = new SimpleMotorFeedforward(0, // kS - minimum voltage to see any movement. AKA "overcome stiction"
                                                0); // kV - Volts required to get one (radian per second) of velocity in steady state

    }

    public void update(boolean forceUpdate){

        // Send the speed command to the motor controller
        //armMotorCtrl.setClosedLoopCmd(desState_speed_radpersec, armMotorFF.calculate(desState_speed_radpersec)); // todo - remove the feed forward for now
        if (forceUpdate || Double.isNaN(prvState_speed_radpersec) || (prvState_speed_radpersec!=desState_speed_radpersec)) {
            armMotorCtrl.setClosedLoopCmd(desState_speed_radpersec, 0); // todo - remove the feed forward for now
            prvState_speed_radpersec = desState_speed_radpersec;
        }

        actState_speed_radpersec = armMotorCtrl.getVelocity_radpersec();

        double currentA = armMotorCtrl.getCurrent_A();

        if (smoothCurrentAmps>1e9) {
            smoothCurrentAmps = currentA;
        }
        else {
            smoothCurrentAmps = smoothCurrentAmps*0.8 + currentA*0.2;
        }
        latestCurrentAmps = currentA;

        armMotorCtrl.update();
    }

   /* Broadcast signals specific to the visualiation
     */
    public void updateTelemetry(){
        double sampleTime = Robot.loopStartTime;
    }

    public void setDesiredState(double speed_radpersec, int currentLimit_A){
        desState_speed_radpersec = speed_radpersec;
        desCurrentLimit_A = currentLimit_A;
    }

    public void setClosedLoopGains(double wheel_kP, double wheel_kI, double wheel_kD, double wheel_kV, double wheel_kS){
        armMotorCtrl.setClosedLoopGains(wheel_kP, wheel_kI, wheel_kD);
        armMotorFF = new SimpleMotorFeedforward(wheel_kS, wheel_kV);
    }

    public void resetWheelEncoder(){
        armMotorCtrl.resetDistance();
    }

    public void setSoftLimits(float fwd, float rev, boolean e) {
        armMotorCtrl.setSoftLimits(fwd, rev, e);
    }

}