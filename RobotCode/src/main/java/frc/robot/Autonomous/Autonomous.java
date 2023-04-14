package frc.robot.Autonomous;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import frc.lib.Autonomous.AutoMode;
import frc.lib.Autonomous.AutoModeList;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.ChargeStationBalancer;
import frc.robot.Arm.ClawControl;
import frc.robot.Arm.ClawControl.AutoClawFingerCtrl;
import frc.robot.Arm.ArmPivotControl;
import frc.robot.Arm.ArmPivotControl.AutoArmPivotCtrl;
import frc.Constants;
import frc.robot.AutoDrive.AutoDrive;
import frc.hardwareWrappers.Gyro.WrapperedGyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
    IntegerTopic curDelayModeTopic;
    IntegerTopic curMainModeTopic;
    IntegerPublisher curDelayModePublisher;
    IntegerPublisher curMainModePublisher;
    IntegerTopic desDelayModeTopic;
    IntegerTopic desMainModeTopic;
    IntegerSubscriber desDelayModeSubscriber;
    IntegerSubscriber desMainModeSubscriber;
    long desDelayMode = 0;
    long desMainMode = 0;
    public AutoModeList mainModeList = new AutoModeList("main");
    public AutoModeList delayModeList = new AutoModeList("delay");
    AutoMode curDelayMode = null;
    AutoMode curMainMode = null;
    AutoMode prevDelayMode = null;
    AutoMode prevMainMode = null;
    ClawControl fng;
    ArmPivotControl armPvt;
    double currentAngle;
    int fsDashboardCurrentState;
    double autoStartTime;
    WrapperedGyro gyro;
    ChargeStationBalancer balancer;
    AutoDrive ad;
    private static final String kAutoNone = "No Auto";
    private static final String kAutoPlace = "Place";
    private static final String kAutoLeave = "Leave";
    private static final String kAutoBalance = "Balance";
    private final SendableChooser < String > m_chooser = new SendableChooser < > ();
    boolean isRunning = false;
    boolean allowedR = false;
    String autoOption = kAutoNone;
    boolean doPlaceStep = false;
    boolean doLeaveStep = false;
    boolean doBalanceStep = false;
    RobotAutoState curState = RobotAutoState.GRABBING_CUBE;
    @Signal
    int curStateInt;
    RobotAutoState prevState = RobotAutoState.WAITING;
    @Signal
    double pitchAngleForPlot = 0;
    @Signal
    double highestAngle = 0;

    /* Singleton infratructure*/
    private static Autonomous inst = null;
    public static synchronized Autonomous getInstance() {
        if (inst == null)
            inst = new Autonomous();
        return inst;
    }

    private Autonomous() {
        m_chooser.setDefaultOption("No Auto", kAutoNone);
        m_chooser.addOption("Place", kAutoPlace);
        m_chooser.addOption("Leave", kAutoLeave);
        m_chooser.addOption("Balance", kAutoBalance);
        SmartDashboard.putData("Auto choices", m_chooser);
        fng = ClawControl.getInstance();
        armPvt = ArmPivotControl.getInstance();
        gyro = WrapperedGyro.getInstance();
        balancer = new ChargeStationBalancer(gyro);
        ad = AutoDrive.getInstance();
    }

    public void reset() {}

    public boolean isActive() {
        return isRunning;
    }

    public void allowRun() {
        autoOption = m_chooser.getSelected();
        System.out.println("autoOption:" + autoOption);
        switch (autoOption) {
            case kAutoNone:
            default:
                allowedR = false;
                doPlaceStep = false;
                doLeaveStep = false;
                doBalanceStep = false;
                break;
            case kAutoPlace:
                allowedR = true;
                doPlaceStep = true;
                doLeaveStep = false;
                doBalanceStep = false;
                break;
            case kAutoLeave:
                allowedR = true;
                doPlaceStep = true;
                doLeaveStep = true;
                doBalanceStep = false;
                break;
            case kAutoBalance:
                allowedR = true;
                doPlaceStep = true;
                doLeaveStep = true;
                doBalanceStep = true;
                break;
        }
    }

    public void blockRun() {
        allowedR = false;
    }
    
    /* This should be called periodically in Disabled, and once in auto init */
    public void sampleDashboardSelector() {}

    public enum RobotAutoState {
        WAITING(0),
            GRABBING_CUBE(1),
            WAITING_A(2),
            TURNING_ARM_60(3),
            OPENING_CLAW(4),
            WAITING_B(5),
            TURNING_ARM_0(6),
            DONE(7),
            BACKING_UP(8),
            WAITING_C(11),
            BACKING_ONTO(10),
            CLIMBING(12),
            BALANCING(9);
        public final int armstate;
        private RobotAutoState(int armstate) {
            this.armstate = armstate;
        }
        public int toInt() {
            return this.armstate;
        }
    }

    public void update() {
        pitchAngleForPlot = balancer.getPitchAngle_deg();
        if (allowedR == true) {
            double currentTime = Timer.getFPGATimestamp();
            double timePassed = currentTime - autoStartTime;
            fsDashboardCurrentState = curState.toInt();
            switch (curState) {
                case WAITING:
                    break;
                case GRABBING_CUBE:
                    if (doPlaceStep) {
                        fng.setFinger(AutoClawFingerCtrl.CLOSE_FOR_CUBE);
                        if (fng.forDashboardCurrentState == 4 || timePassed > 1) {
                            curState = RobotAutoState.WAITING_A;
                        }
                    } else {
                        curState = RobotAutoState.DONE;
                    }
                    break;
                case WAITING_A:
                    if (timePassed > 0.25) {
                        curState = RobotAutoState.TURNING_ARM_60;
                    }
                    break;
                case TURNING_ARM_60:

                    double angle = armPvt.absoluteAngleArm_deg;
                    if (angle < -65) {
                        armPvt.setArmSpeed(0.7 * Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
                        armPvt.setArm(AutoArmPivotCtrl.MOVE_UP);
                    } else if (angle > -59) {
                        armPvt.setArmSpeed(-0.9 * Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
                        armPvt.setArm(AutoArmPivotCtrl.MOVE_DOWN);
                    } else {
                        armPvt.setArmSpeed(0);
                        armPvt.setArm(AutoArmPivotCtrl.STOP_MOVEMT);
                        curState = RobotAutoState.OPENING_CLAW;
                    }
                    break;
                case OPENING_CLAW:
                    fng.setFinger(AutoClawFingerCtrl.OPEN_PHALANGES);
                    curState = RobotAutoState.WAITING_B;
                    break;
                case WAITING_B:
                    if (timePassed > 0.3) {
                        curState = RobotAutoState.TURNING_ARM_0;
                    }
                    break;
                case TURNING_ARM_0:
                case BACKING_UP:
                    if (doLeaveStep) {
                        curState = RobotAutoState.BACKING_UP;
                    }
                    angle = armPvt.absoluteAngleArm_deg;
                    if (angle <= -2) {
                        armPvt.setArmSpeed(0.9 * Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
                        armPvt.setArm(AutoArmPivotCtrl.MOVE_UP);
                    } else {
                        armPvt.setArmSpeed(0);
                        armPvt.setArm(AutoArmPivotCtrl.STOP_MOVEMT);
                        if (!doLeaveStep) {
                            curState = RobotAutoState.DONE;
                        }
                    }
                    if (curState == RobotAutoState.BACKING_UP) {
                        double speed = 0.24;
                        if (Math.abs(balancer.getPitchAngle_deg()) > highestAngle) {
                            highestAngle = Math.abs(balancer.getPitchAngle_deg());
                        }
                        if (balancer.getPitchAngle_deg() < -2) {
                            speed = 0.30;
                        }
                        if (balancer.getPitchAngle_deg() < -5) {
                            speed = 0.25;
                        }
                        if (balancer.getPitchAngle_deg() < -7) {
                            speed = 0.26;
                        }
                        if (balancer.getPitchAngle_deg() < -9) {
                            speed = 0.27;
                        }
                        if (balancer.getPitchAngle_deg() < -11) {
                            speed = 0.28;
                        }
                        if (balancer.getPitchAngle_deg() > 3) {
                            speed = 0.27;
                        }
                        if (balancer.getPitchAngle_deg() > 5) {
                            speed = 0.26;
                        }
                        currentTime = Timer.getFPGATimestamp();
                        timePassed = currentTime - autoStartTime;
                        if (timePassed > 3.6) {
                            if (doBalanceStep) {
                                curState = RobotAutoState.WAITING_C;
                            } else {
                                curState = RobotAutoState.DONE;
                            }
                        }
                        ad.transformRobot(-speed, 0, 0, true);
                    }
                    break;
                case WAITING_C:
                    ad.transformRobot(0, 0, 0, true);
                    if (timePassed > 0.3) {
                        if (armPvt.absoluteAngleArm_deg > -2) {
                            curState = RobotAutoState.BACKING_ONTO;
                            armPvt.setArmSpeed(0);
                            armPvt.setArm(AutoArmPivotCtrl.STOP_MOVEMT);
                        } else {
                            armPvt.setArmSpeed(0.9 * Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
                            armPvt.setArm(AutoArmPivotCtrl.MOVE_UP);
                        }
                    }
                    break;
                case BACKING_ONTO:
                    boolean chargeStation = true;
                    double speeds = 0.20;
                    if (chargeStation) {
                        if (balancer.getPitchAngle_deg() > 5) {
                            speeds = 0.22;
                        }
                        if (balancer.getPitchAngle_deg() > 6) {
                            speeds = 0.25;
                        }
                        if (balancer.getPitchAngle_deg() > 13) {
                            curState = RobotAutoState.CLIMBING;
                        }
                    } else {
                        if (timePassed > 1) {
                            curState = RobotAutoState.DONE;
                        }
                    }
                    ad.transformRobot(speeds, 0, 0, true);
                    break;
                case CLIMBING:
                    double speeder = 0.23;
                    ad.transformRobot(speeder, 0, 0, true);
                    if (balancer.getPitchAngle_deg() < 12) {
                        curState = RobotAutoState.BALANCING;
                    }
                    break;
                case BALANCING:
                    balancer.balanceOnChargeStation();
                    break;
                case DONE:
                    ad.transformRobot(0, 0, 0, true);
                    break;
            }
            if (prevState != curState) {
                autoStartTime = Timer.getFPGATimestamp();
            }
            curStateInt = curState.toInt();
            prevState = curState;
        }
    }
}