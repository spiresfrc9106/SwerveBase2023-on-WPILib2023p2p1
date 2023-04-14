package frc.robot.Autonomous;
import edu.wpi.first.math.util.Units;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.CenterCamera;
import frc.robot.ChargeStationBalancer;
import frc.robot.Arm.ClawControl;
import frc.robot.Arm.ArmPivotControl;
import frc.robot.AutoDrive.AutoDrive;

public class AutoPickupSystem {
    double currentAngle;
    int fsDashboardCurrentState;
    double autoStartTime;
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
    
    private AutoPickupSystem() {
        fng = ClawControl.getInstance();
        armPvt = ArmPivotControl.getInstance();
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
    
    public void update(double sonarDistance_in) {
        exeCount = exeCount + 1;
        seesTarget = cc.camera.seesTarget();
        if (seesTarget == 1) {
            seesTarget_int = 1;
        } else {
            seesTarget_int = 0;
        }
        double propGainForRotation = 0.02 * 2;
        double propGainForForward = 0.02 * 2 * 3 * 3;
        if (seesTarget == 1) { //sees target\
            if (Math.abs(cc.camera.getX()) > 4) { //change to 3
                azVelocity = -cc.camera.getX() * propGainForRotation;
            } else {
                azVelocity = 0;
            }
            if (Math.abs(cc.camera.getX()) < 4) {
                double distanceNeeded_m = Units.inchesToMeters(sonarDistance_in - 37);
                fwdVelocity_mps = -distanceNeeded_m * propGainForForward;
            }
        } else {
            azVelocity = 0;
        }
    }
}