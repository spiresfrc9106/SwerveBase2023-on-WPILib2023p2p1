package frc.robot.Arm;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.Constants;
import frc.lib.Calibration.Calibration;

public class ClawControl {
    /* Singleton infrastructure */
    private static ClawControl instance;
    public static ClawControl getInstance() {
        if (instance == null) {
            instance = new ClawControl();
        }
        return instance;
    }

    frc.lib.Signal.Signal clawStateSig; // The two claw finger modules under our control
    ClawFingerControl moduleL;
    ClawFingerControl moduleR;
    Calibration moduleClaw_kP;
    Calibration moduleClaw_kI;
    Calibration moduleClaw_kD;
    Calibration moduleClaw_kV; // Feed-forward Voltage Constat - IE, how many volts to get a certain amount of motor speed in radians per second?
    Calibration moduleClaw_kS; // Feed-forward Static Constnat - IE, how many volts to overcome static friction and get any motion at all?
    ChassisSpeeds desFingSpd = new ChassisSpeeds(0, 0, 0);
    double desFingSpdVx;
    double desFingSpdVy;
    double desFingSpdOmega;
    AutoClawFingerCtrl curCmd = AutoClawFingerCtrl.STOP_PHALANGES;
    AutoFingerState curState = AutoFingerState.STOP_PHALANGES;
    AutoFingerState prevState = AutoFingerState.STOP_PHALANGES;
    public int forDashboardCurrentState = AutoClawFingerCtrl.STOP_PHALANGES.toInt();

    private ClawControl() {
        moduleClaw_kP = new Calibration("Claw Module Motor kP", "", 0.0015);
        moduleClaw_kI = new Calibration("Claw Module Motor kI", "", 0.0);
        moduleClaw_kD = new Calibration("Claw Module Motor kD", "", 0.0);
        moduleClaw_kV = new Calibration("Claw Module Motor kV", "volts/radPerSec", 0.018);
        moduleClaw_kS = new Calibration("Claw Module Motor kS", "volts", 0.172);
        moduleL = new ClawFingerControl("L", Constants.L_FINGER_MOTOR_CANID, Constants.INVERT_FINGER_DIRECTION[Constants.L_FINGER]);
        moduleR = new ClawFingerControl("R", Constants.R_FINGER_MOTOR_CANID, Constants.INVERT_FINGER_DIRECTION[Constants.R_FINGER]);
        calUpdate(true);
    }

    // Pass the current calibration values downard into child classes.
    public void calUpdate(boolean force) {
        // Guard these Cal updates with isChanged because they write to motor controllers
        // and that soaks up can bus bandwidth, which we don't want.
        // There's probably a better and clearner way to do this, but it works for now.
        if (moduleClaw_kP.isChanged() ||
            moduleClaw_kI.isChanged() ||
            moduleClaw_kD.isChanged() ||
            moduleClaw_kV.isChanged() ||
            moduleClaw_kS.isChanged() || force) {
            moduleL.setClosedLoopGains(moduleClaw_kP.get(), moduleClaw_kI.get(), moduleClaw_kD.get(), moduleClaw_kV.get(), moduleClaw_kS.get());
            moduleR.setClosedLoopGains(moduleClaw_kP.get(), moduleClaw_kI.get(), moduleClaw_kD.get(), moduleClaw_kV.get(), moduleClaw_kS.get());
            moduleClaw_kP.acknowledgeValUpdate();
            moduleClaw_kI.acknowledgeValUpdate();
            moduleClaw_kD.acknowledgeValUpdate();
            moduleClaw_kV.acknowledgeValUpdate();
            moduleClaw_kS.acknowledgeValUpdate();
        }
    }

    // Cause all non-annotated signals to broadcast a new value for the loop.
    public void updateTelemetry() {
        moduleL.updateTelemetry();
        moduleR.updateTelemetry();
    }

    public enum AutoClawFingerCtrl {
        OPEN_PHALANGES(0),
        CLOSE_FOR_CUBE(1),
        CLOSE_FOR_CONE(2),
        STOP_PHALANGES(3),
        KEEP_MOVING(4);
        public final int fingerstate;
        private AutoClawFingerCtrl(int fingerstate) {
            this.fingerstate = fingerstate;
        }
        public int toInt() {
            return this.fingerstate;
        }
    }

    public enum AutoFingerState {
        OPEN_PHALANGES(0),
        CLOSE_FOR_CUBE(1),
        CLOSE_FOR_CONE(2),
        STOP_PHALANGES(3),
        CURRENT_LIMIT_CLOSE(4),
        CURRENT_LIMIT_OPEN(5);
        public final int fingerstate;
        private AutoFingerState(int fingerstate) {
            this.fingerstate = fingerstate;
        }
        public int toInt() {
            return this.fingerstate;
        }
    }

    public void setFinger(AutoClawFingerCtrl cmd) {
        curCmd = cmd;
    }

    public void update() {
        boolean inOvercurrent = false; 
        double ml = moduleL.smoothCurrentAmps;
        double mr = moduleR.smoothCurrentAmps;
        double hi = 0;
        boolean nextStateIfOverIsOpen = true;
        if (ml > mr) {
            hi = ml;
        } else {
            hi = mr;
        }
        if (prevState == AutoFingerState.OPEN_PHALANGES) {
            if (hi > Constants.OPEN_PHALANGES_CURRENT_A) {
                inOvercurrent = true;
                nextStateIfOverIsOpen = true;
            } else {
                inOvercurrent = false;
            }
        }
        if (prevState == AutoFingerState.CLOSE_FOR_CONE) {
            if (hi > Constants.CLOSE_FOR_CONE_CURRENT_A) {
                inOvercurrent = true;
                nextStateIfOverIsOpen = false;
            } else {
                inOvercurrent = false;
            }
        }
        if (prevState == AutoFingerState.CLOSE_FOR_CUBE) {
            if (hi > Constants.CLOSE_FOR_CUBE_CURRENT_A) {
                inOvercurrent = true;
                nextStateIfOverIsOpen = false;
            } else {
                inOvercurrent = false;
            }
        }
        if (inOvercurrent) {
            if (nextStateIfOverIsOpen) {
                curState = AutoFingerState.CURRENT_LIMIT_OPEN;
            } else {
                curState = AutoFingerState.CURRENT_LIMIT_CLOSE;
            }
        } else {
            switch (prevState) {
                case OPEN_PHALANGES:
                case CLOSE_FOR_CUBE:
                case CLOSE_FOR_CONE:
                case STOP_PHALANGES:
                    switch (curCmd) {
                        case OPEN_PHALANGES:
                            curState = AutoFingerState.OPEN_PHALANGES;
                            break;
                        case CLOSE_FOR_CUBE:
                            curState = AutoFingerState.CLOSE_FOR_CUBE;
                            break;
                        case CLOSE_FOR_CONE:
                            curState = AutoFingerState.CLOSE_FOR_CONE;
                            break;
                        case STOP_PHALANGES:
                            curState = AutoFingerState.STOP_PHALANGES;
                            break;
                        case KEEP_MOVING:
                            curState = prevState;
                        default:
                            curState = AutoFingerState.STOP_PHALANGES;
                    }
                    break;
                case CURRENT_LIMIT_CLOSE:
                    switch (curCmd) {
                        case OPEN_PHALANGES:
                            curState = AutoFingerState.OPEN_PHALANGES;
                            break;
                        case CLOSE_FOR_CUBE:
                        case CLOSE_FOR_CONE:
                        case STOP_PHALANGES:
                        default:
                            curState = AutoFingerState.CURRENT_LIMIT_CLOSE;
                            break;
                    }
                    break;
                case CURRENT_LIMIT_OPEN:
                    switch (curCmd) {
                        case OPEN_PHALANGES:
                        case STOP_PHALANGES:
                        default:
                            curState = AutoFingerState.CURRENT_LIMIT_OPEN;
                            break;
                        case CLOSE_FOR_CUBE:
                            curState = AutoFingerState.CLOSE_FOR_CUBE;
                            break;
                        case CLOSE_FOR_CONE:
                            curState = AutoFingerState.CLOSE_FOR_CONE;
                            break;
                    }
                    break;
                default:
                    curState = AutoFingerState.STOP_PHALANGES;
            }
        }
        forDashboardCurrentState = curState.toInt();
        if (curState != prevState) {
            double default_speed_radpersec = +Math.PI * 2.0 * 2.5 * 3.0;
            double speed_radpersec = 0;
            int currentLimit_A = 0;
            switch (curState) {
                case OPEN_PHALANGES:
                    speed_radpersec = default_speed_radpersec;
                    currentLimit_A = Constants.OPEN_PHALANGES_CURRENT_A;
                    break;
                case CLOSE_FOR_CUBE:
                    speed_radpersec = -default_speed_radpersec;
                    currentLimit_A = Constants.CLOSE_FOR_CUBE_CURRENT_A;
                    break;
                case CLOSE_FOR_CONE:
                    speed_radpersec = -default_speed_radpersec * 1.35;
                    currentLimit_A = Constants.CLOSE_FOR_CONE_CURRENT_A;
                    break;
                case STOP_PHALANGES:
                    speed_radpersec = 0;
                    currentLimit_A = Constants.STOP_PHALANGES_CURRENT_A;
                    break;
                case CURRENT_LIMIT_CLOSE:
                case CURRENT_LIMIT_OPEN:
                default:
            }
            switch (curState) {
                case OPEN_PHALANGES:
                case CLOSE_FOR_CUBE:
                case CLOSE_FOR_CONE:
                case STOP_PHALANGES:
                    moduleL.setDesiredState(speed_radpersec, currentLimit_A);
                    moduleR.setDesiredState(speed_radpersec, currentLimit_A);
                    break;
                case CURRENT_LIMIT_CLOSE:
                    moduleL.setDesiredState(-default_speed_radpersec, currentLimit_A);
                    moduleR.setDesiredState(-default_speed_radpersec, currentLimit_A);
                    break;
                case CURRENT_LIMIT_OPEN:
                    moduleL.setDesiredState(default_speed_radpersec, currentLimit_A);
                    moduleR.setDesiredState(default_speed_radpersec, currentLimit_A);
                    break;
                default:
            }
        }
        moduleL.update();
        moduleR.update();
        prevState = curState;
    }
}