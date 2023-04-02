package frc.robot.Arm;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;

public class ClawFingerControl {

    WrapperedCANMotorCtrl fingerMotorCtrl;

    double desState_speed_radpersec;
    double prvState_speed_radpersec = Double.NaN;
    double actState_speed_radpersec;

    int desCurrentLimit_A;
    int prvCurrentLimit_A=0;

    double motorDesSpd_radpersec;

    //frc.lib.Signal.Signal fingSpdDesSig;
    //frc.lib.Signal.Signal fingSpdActSig;
    //frc.lib.Signal.Signal fingCurActSig;

    SimpleMotorFeedforward fingerMotorFF;

    //@Signal (units="double")
    public double smoothCurrentAmps = 1e10;
    //@Signal (units="double")
    public double latestCurrentAmps;

    public ClawFingerControl(String fingerName, int fingerMotorIdx, boolean invertFinger){
        prvCurrentLimit_A = 20;
        fingerMotorCtrl = new WrapperedCANMotorCtrl("finger"+fingerName, fingerMotorIdx, WrapperedCANMotorCtrl.CANMotorCtrlType.SPARK_MAX, prvCurrentLimit_A);
        
        //String topic_prefix = "finger";
        fingerMotorCtrl.setInverted(invertFinger);
        //fingSpdDesSig = new frc.lib.Signal.Signal(topic_prefix + fingerName + "speedDes", "RPM");
        //fingSpdActSig = new frc.lib.Signal.Signal(topic_prefix + fingerName + "speedAct", "RPM");
        //fingCurActSig = new frc.lib.Signal.Signal(topic_prefix + fingerName + "currentAct", "A");

        fingerMotorFF = new SimpleMotorFeedforward(0, // kS - minimum voltage to see any movement. AKA "overcome stiction"
                                                  0); // kV - Volts required to get one (radian per second) of velocity in steady state

    }

    public void update(){ 

        // Send the speed command to the motor controller
        if (Double.isNaN(prvState_speed_radpersec) || (prvState_speed_radpersec!=desState_speed_radpersec)) {
            //System.out.printf("Des Speed=%f\n", desState_speed_radpersec);
            fingerMotorCtrl.setClosedLoopCmd(desState_speed_radpersec, fingerMotorFF.calculate(desState_speed_radpersec)); // todo - remove the feed forward for now
            prvState_speed_radpersec = desState_speed_radpersec;
        }
        if (prvCurrentLimit_A!=desCurrentLimit_A) {
            //System.out.printf("Des Curent=%d\n",desCurrentLimit_A);
            fingerMotorCtrl.setCurrent_A(desCurrentLimit_A);
            prvCurrentLimit_A=desCurrentLimit_A;
        }

        fingerMotorCtrl.update();
        actState_speed_radpersec = fingerMotorCtrl.getVelocity_radpersec();

        double currentA = fingerMotorCtrl.getCurrent_A();

        if (smoothCurrentAmps>1e9) {
            smoothCurrentAmps = currentA;
        }
        else {
            smoothCurrentAmps = smoothCurrentAmps*0.8 + currentA*0.2;
        }
        latestCurrentAmps = currentA;
    }

   /* Broadcast signals specific to the visualiation
     */
    public void updateTelemetry(){
        //double sampleTime = Robot.loopStartTime;

        //fingSpdDesSig.addSample(sampleTime, desState_speed_radpersec); 
        //fingSpdActSig.addSample(sampleTime, actState_speed_radpersec);
        //fingCurActSig.addSample(sampleTime, smoothCurrentAmps);
    }

    public void setDesiredState(double speed_radpersec, int currentLimit_A){
        desState_speed_radpersec = speed_radpersec;
        desCurrentLimit_A = currentLimit_A;
    }

    public void setClosedLoopGains(double wheel_kP, double wheel_kI, double wheel_kD, double wheel_kV, double wheel_kS){
        fingerMotorCtrl.setClosedLoopGains(wheel_kP, wheel_kI, wheel_kD);
        fingerMotorFF = new SimpleMotorFeedforward(wheel_kS, wheel_kV);
    }

    public void resetWheelEncoder(){
        fingerMotorCtrl.resetDistance();
    }

}