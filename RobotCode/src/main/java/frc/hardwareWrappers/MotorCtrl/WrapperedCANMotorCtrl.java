package frc.hardwareWrappers.MotorCtrl;

import frc.hardwareWrappers.MotorCtrl.Sim.SimSmartMotor;
import frc.hardwareWrappers.MotorCtrl.SparkMax.RealSparkMax;
import frc.hardwareWrappers.MotorCtrl.TalonFX.RealTalonFX;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.Robot;

public class WrapperedCANMotorCtrl {

    public enum CANMotorCtrlType {
        TALON_FX,
        SPARK_MAX
    }

    AbstractSimmableMotorController ctrl;

    @Signal(units="V")
    private double appliedVoltage;

    @Signal(units = "radpersec")
    private double actVel;

    @Signal(units = "A")
    private double current;

    @Signal(units = "radpersec")
    private double desVel;

    @Signal(units = "rad")
    private double actPos;

    public WrapperedCANMotorCtrl(String prefix, int can_id, CANMotorCtrlType type, int currentLimitAmps){

        System.out.print("=> Starting motor controller init for " + prefix + " CANID = " + Integer.toString(can_id));

        if(Robot.isSimulation()){
            ctrl = new SimSmartMotor(can_id);
        } else {
            switch(type){
                case TALON_FX:
                    ctrl = new RealTalonFX(can_id, currentLimitAmps);
                    break;
                case SPARK_MAX:
                    ctrl = new RealSparkMax(can_id, currentLimitAmps);
                    break;
            }
        }
        System.out.println(" ... Done!");

    }
    

    public void update(){
        actVel = ctrl.getVelocity_radpersec();
        actPos = ctrl.getPosition_rad();
        current = ctrl.getCurrent_A();
        appliedVoltage = ctrl.getAppliedVoltage_V();
    }

    public void setCurrent_A(int currentLimit_A) {
        ctrl.setCurrent_A(currentLimit_A);
    }

    public void setSoftLimits(float fwd, float rev, boolean e) {
        
        ctrl.setSoftLimits(fwd, rev, e);
    }

    public void setInverted(boolean invert){
        ctrl.setInverted(invert);
    }

    public void setClosedLoopGains(double p, double i, double d){
        ctrl.setClosedLoopGains(p, i, d);
    }

    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V){
        ctrl.setClosedLoopCmd(velocityCmd_radpersec, arbFF_V);
        desVel = velocityCmd_radpersec;
    }

    public void setVoltageCmd(double cmd_V){
        ctrl.setVoltageCmd(cmd_V);
    }

    public double getCurrent_A(){
        return current;
    }

    public double getVelocity_radpersec(){
        return actVel;
    }

    public double getPosition_rad(){
        return actPos;
    }

    public void resetDistance(){
        ctrl.resetDistance();
    }

    public double getAppliedVoltage_V() {
        return ctrl.getAppliedVoltage_V();
    }

}
