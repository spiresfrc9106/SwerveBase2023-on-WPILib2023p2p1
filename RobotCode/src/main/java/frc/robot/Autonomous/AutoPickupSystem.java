package frc.robot.Autonomous;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.lib.Autonomous.AutoModeList;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Util.CrashTracker;
import frc.robot.CenterCamera;
import frc.robot.ChargeStationBalancer;
import frc.robot.PoseTelemetry;
import frc.robot.Robot;
import frc.robot.Drivetrain.DrivetrainControl;
import frc.robot.Arm.ClawControl;
import frc.robot.Arm.ClawControl.AutoClawFingerCtrl;
import frc.robot.Arm.ArmPivotControl;
import frc.robot.Arm.ArmPivotControl.AutoArmPivotCtrl;
import frc.Constants;

import javax.swing.plaf.synth.SynthTextAreaUI;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.AutoDrive.AutoDrive;
import frc.hardwareWrappers.Gyro.WrapperedGyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoPickupSystem {
    double currentAngle;
    int fsDashboardCurrentState;
    double autoStartTime;

    //WrapperedGyro gyro;
    ChargeStationBalancer balancer;
    AutoDrive ad;
    ClawControl fng;
    ArmPivotControl armPvt;

    CenterCamera cc;

    /* Singleton infratructure*/
    private static AutoPickupSystem inst = null;
    public static synchronized AutoPickupSystem getInstance() {
        if (inst == null)
            inst = new AutoPickupSystem();
        return inst;
    }


    private AutoPickupSystem(){
        fng = ClawControl.getInstance();
        armPvt = ArmPivotControl.getInstance();
        //gyro = WrapperedGyro.getInstance();
        //balancer = new ChargeStationBalancer(gyro);
        ad = AutoDrive.getInstance();
        cc = new CenterCamera();
    }

    public void reset() {}
    @Signal
    public double azVelocity = 0;
    @Signal
    public double fwdVelocity_mps = 0;
    @Signal
    int seesTarget_int;
    long seesTarget;
    @Signal
    int exeCount = 0;

    /* This should be called periodically, always */
    public void update(double sonarDistance_in) {
        exeCount = exeCount+1;
        seesTarget = cc.camera.seesTarget();
        if (seesTarget==1) {
            seesTarget_int = 1;
        }
        else {
            seesTarget_int = 0;
        }
        double propGainForRotation = 0.02*2;
        double propGainForForward = 0.02*2*3*3;

        if (seesTarget==1) { //sees target\
            if (Math.abs(cc.camera.getX())>4) { //change to 3
                azVelocity = - cc.camera.getX()*propGainForRotation;
            }
            else {
                azVelocity = 0;
            }
            if (Math.abs(cc.camera.getX())<4) {
                double distanceNeeded_m = Units.inchesToMeters(sonarDistance_in-37);
                fwdVelocity_mps = - distanceNeeded_m*propGainForForward;
            }
        }
        else {
            azVelocity = 0;
        }
/*
        pitchAngleForPlot = balancer.getPitchAngle_deg();
        if (true==true) {
            double currentTime = Timer.getFPGATimestamp();
            double timePassed = currentTime-autoStartTime;

            fsDashboardCurrentState = curState.toInt();
 */
            /*
            switch (curState) {
                case WAITING:
                    autoStartTime = Timer.getFPGATimestamp();
                break;
                case GRABBING_CUBE:
                    if (true) {
                        fng.setFinger(AutoClawFingerCtrl.CLOSE_FOR_CUBE);
                        if (fng.forDashboardCurrentState==4 || timePassed>1) {
                            curState = RobotAutoState.WAITING_A;
                            autoStartTime = Timer.getFPGATimestamp();
                        }
                    } else {
                        curState = RobotAutoState.DONE;
                    }
                break;
                case WAITING_A:
                    //System.out.println(timePassed);
                    if (timePassed>0.25) {
                        curState = RobotAutoState.TURNING_ARM_60;
                    }
                break;
                case TURNING_ARM_60:

                    double angle = armPvt.absoluteAngleArm_deg;
                    if (angle<-65) {
                        armPvt.setArmSpeed(0.7*Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
                        armPvt.setArm(AutoArmPivotCtrl.MOVE_UP);
                    } else if (angle>-59) {
                        armPvt.setArmSpeed(-0.9*Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
                        armPvt.setArm(AutoArmPivotCtrl.MOVE_DOWN);
                    } else {
                        armPvt.setArmSpeed(0);
                        armPvt.setArm(AutoArmPivotCtrl.STOP_MOVEMT);
                        curState = RobotAutoState.OPENING_CLAW;
                    }
                    //System.out.println("current angle: "+angle);
                    break;
                case OPENING_CLAW:
                        fng.setFinger(AutoClawFingerCtrl.OPEN_PHALANGES);
                        curState = RobotAutoState.WAITING_B;
                        autoStartTime = Timer.getFPGATimestamp();
                        break;
                case WAITING_B:
                    //System.out.println(timePassed);
                    if (timePassed>0.3) {
                        curState = RobotAutoState.TURNING_ARM_0;
                    } 
                break;
                case TURNING_ARM_0:
                case BACKING_UP:
                    if (true) {
                        curState = RobotAutoState.BACKING_UP;
                    } 
                    angle = armPvt.absoluteAngleArm_deg;
                    if (angle<=-2) {
                        armPvt.setArmSpeed(0.9*Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
                        armPvt.setArm(AutoArmPivotCtrl.MOVE_UP);
                    } else {
                        armPvt.setArmSpeed(0);
                        armPvt.setArm(AutoArmPivotCtrl.STOP_MOVEMT);
                        if (!true) {
                            curState = RobotAutoState.DONE;
                        }
                    }
                    if (curState==RobotAutoState.BACKING_UP) {
                        double speed = 0.24;
                        if (Math.abs(balancer.getPitchAngle_deg())>highestAngle) {
                            highestAngle = Math.abs(balancer.getPitchAngle_deg());
                        }
                        if (balancer.getPitchAngle_deg()< -2) {
                            speed = 0.30;
                        }
                        if (balancer.getPitchAngle_deg()< -5) {
                            speed = 0.25;
                        }
                        if (balancer.getPitchAngle_deg()< -7) {
                            speed = 0.26;
                        }
                        if (balancer.getPitchAngle_deg()< -9) {
                            speed = 0.27;
                        }
                        if (balancer.getPitchAngle_deg()< -11) {
                            speed = 0.28;
                        }
                        if (balancer.getPitchAngle_deg()> 3) {
                            speed = 0.27;
                        }
                        if (balancer.getPitchAngle_deg()> 5) {
                            speed = 0.26;
                        }
                        currentTime = Timer.getFPGATimestamp();
                        timePassed = currentTime-autoStartTime;
                        if (timePassed>3.6) {
                            if (true) {
                                curState = RobotAutoState.WAITING_C;                           
                            } else {
                                curState = RobotAutoState.DONE;
                            }
                        }
                        //System.out.println("speed m: "+speed);
                        ad.transformRobot(-speed,0,0,true);
                        }
                break;
                case WAITING_C:
                    ad.transformRobot(0,0,0,true);
                    if (timePassed>0.2) {
                        if (armPvt.absoluteAngleArm_deg>-2) {
                            curState = RobotAutoState.BACKING_ONTO;
                            armPvt.setArmSpeed(0);
                            armPvt.setArm(AutoArmPivotCtrl.STOP_MOVEMT);
                        } else {
                            armPvt.setArmSpeed(0.9*Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
                            armPvt.setArm(AutoArmPivotCtrl.MOVE_UP);
                        }
                    }
                break;
                case BACKING_ONTO:
                    boolean chargeStation = true;
                    double speeds = 0.20;
                    if (chargeStation) {
                        if (balancer.getPitchAngle_deg()> 5) {
                            speeds = 0.22;
                        }
                        if (balancer.getPitchAngle_deg()> 6) {
                            speeds = 0.25;
                        }
                        if (balancer.getPitchAngle_deg()> 17) {
                            curState = RobotAutoState.CLIMBING;
                        }
                    }
                    else {
                        if (timePassed>1) {
                            curState = RobotAutoState.DONE;
                        }
                    }
                    ad.transformRobot(speeds,0,0,true);
                break;
                case CLIMBING:
                    double speeder = 0.25;
                    ad.transformRobot(speeder,0,0,true);
                    if (balancer.getPitchAngle_deg()< 12) {
                        curState = RobotAutoState.BALANCING;
                    }
                break;
                case BALANCING:
                    boolean isBalanced = balancer.balanceOnChargeStation();
                    System.out.println("Balancing");
                break;
                case DONE:
                    ad.transformRobot(0,0,0,true);
                break;
            }
            if (prevState!=curState) {
                autoStartTime = Timer.getFPGATimestamp();
                System.out.println("reset timer");
                System.out.println("switched from "+prevState+" to "+curState);
            }
            curStateInt = curState.toInt();
            prevState = curState;
        }
         */
    }

}