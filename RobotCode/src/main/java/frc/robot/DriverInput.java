package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import frc.Constants;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

public class DriverInput {
    XboxController driverController;
    SlewRateLimiter fwdRevSlewLimiter;
    SlewRateLimiter rotSlewLimiter;
    SlewRateLimiter sideToSideSlewLimiter;
    Calibration stickDeadband;
    Calibration fwdRevSlewRate;
    Calibration rotSlewRate;
    Calibration sideToSideSlewRate;
    Calibration translateCmdScalar;
    Calibration rotateCmdScalar;
    @Signal(units = "cmd")
    double curFwdRevCmd;
    @Signal(units = "cmd")
    double curRotCmd;
    @Signal(units = "cmd")
    double curSideToSideCmd;
    @Signal(units = "cmd")
    double armUpCmd;
    @Signal(units = "cmd")
    double armDownCmd;
    @Signal(units = "cmd")
    double armFinalCmd;
    @Signal(units = "cmd")
    double armWithSpeed;
    @Signal(units = "bool")
    boolean robotRelative;
    @Signal(units = "bool")
    boolean alignRobotDir;
    @Signal(units = "cmd")
    double fwdRevSlewCmd;
    @Signal(units = "cmd")
    double rotSlewCmd;
    @Signal(units = "cmd")
    double sideToSideSlewCmd;
    boolean stopSoftLimits;
    @Signal(units = "bool")
    boolean clawOpen;
    @Signal(units = "bool")
    boolean clawCube;
    @Signal(units = "bool")
    boolean clawCone;
    boolean clawStop;
    boolean armDown;
    boolean armUp;
    boolean armFront;
    boolean armBack;
    @Signal(units = "bool")
    boolean presetU;
    @Signal(units = "bool")
    boolean presetR;
    @Signal(units = "bool")
    boolean presetD;
    @Signal(units = "bool")
    boolean presetL;
    @Signal(units = "bool")
    boolean changeToPreset;
    boolean spinMoveCmd;
    @Signal(units = "bool")
    boolean driveToCenterCmd;
    @Signal(units = "bool")
    boolean resetOdometry;
    @Signal(units = "bool")
    boolean isConnected;
    @Signal(units = "double")
    double speedFactor;
    @Signal(units = "deg")
    double currentAngle;
    @Signal(units = "deg")
    double changeTo;
    @Signal(units = "deg")
    double methOne;
    @Signal(units = "bool")
    boolean matchedBasically;
    String methTwo;
    @Signal(units = "bool")
    public boolean doAutoObjectAlign = false;
    boolean leftStickButtonCode;
    boolean rightStickButtonCode;
    Debouncer resetOdoDbnc = new Debouncer(0.5, DebounceType.kRising);

    String getName(int idx) {
        return "Driver Ctrl " + Integer.toString(idx) + " ";
    }

    public DriverInput(int controllerIdx) {
        driverController = new XboxController(controllerIdx);
        stickDeadband = new Calibration(getName(controllerIdx) + "StickDeadBand", "", 0.1);
        fwdRevSlewRate = new Calibration(getName(controllerIdx) + "fwdRevSlewRate_", "", 1);
        rotSlewRate = new Calibration(getName(controllerIdx) + "rotSlewRate", "", 1);
        sideToSideSlewRate = new Calibration(getName(controllerIdx) + "sideToSideSlewRate", "", 1);
        translateCmdScalar = new Calibration(getName(controllerIdx) + "translateCmdScalar", "", 0.8);
        rotateCmdScalar = new Calibration(getName(controllerIdx) + "rotateCmdScalar", "", 0.8);
        fwdRevSlewLimiter = new SlewRateLimiter(fwdRevSlewRate.get());
        rotSlewLimiter = new SlewRateLimiter(rotSlewRate.get());
        sideToSideSlewLimiter = new SlewRateLimiter(sideToSideSlewRate.get());
    }

    public void update() {
        isConnected = driverController.isConnected();
        int id = driverController.getPort();
        boolean isDriver = false;
        boolean isPicker = false;
        if (id == 0) {
            isDriver = true;
            isPicker = false;
        } else if (id == 1) {
            isDriver = false;
            isPicker = true;
        }
        if (isConnected) {
            curFwdRevCmd = Constants.SWERVE_FWD_REV_CMD_SIGN * driverController.getLeftY() / 2.0;
            curRotCmd = Constants.SWERVE_ROT_CMD_SIGN * driverController.getRightX() / 2.0;
            curSideToSideCmd = Constants.SWERVE_SIDE_TO_SIDE_CMD_SIGN * driverController.getLeftX() / 2.0;
            if ((isPicker) && (driverController.getBackButton() == true) && (driverController.getStartButton() == true)) {
                stopSoftLimits = true;
            } else {
                stopSoftLimits = false;
            }
            if (driverController.getLeftBumper()) {
                curFwdRevCmd = curFwdRevCmd * 2.0;
                curSideToSideCmd = curSideToSideCmd * 2.0;
                curRotCmd = curRotCmd * 2.0;
            } else {
                curFwdRevCmd = curFwdRevCmd * (13.0 / 19.0);
                curSideToSideCmd = curSideToSideCmd * (13.0 / 19.0);
            }
            if (isDriver) {
                int moveAngle = driverController.getPOV();
                if (moveAngle == -1) {}
                else {
                    double power = 0.25;
                    if (driverController.getLeftBumper()) {
                        power = 1.0;
                    }
                    if (moveAngle == 0) {
                        curFwdRevCmd = 1;
                        curSideToSideCmd = 0;
                    }
                    if (moveAngle == 45) {
                        curFwdRevCmd = 1;
                        curSideToSideCmd = 1;
                    }
                    if (moveAngle == 90) {
                        curFwdRevCmd = 0;
                        curSideToSideCmd = 1;
                    }
                    if (moveAngle == 135) {
                        curFwdRevCmd = -1;
                        curSideToSideCmd = 1;
                    }
                    if (moveAngle == 180) {
                        curFwdRevCmd = -1;
                        curSideToSideCmd = 0;
                    }
                    if (moveAngle == 225) {
                        curFwdRevCmd = -1;
                        curSideToSideCmd = -1;
                    }
                    if (moveAngle == 270) {
                        curFwdRevCmd = 0;
                        curSideToSideCmd = -1;
                    }
                    if (moveAngle == 315) {
                        curFwdRevCmd = 1;
                        curSideToSideCmd = -1;
                    }
                    curFwdRevCmd = Constants.SWERVE_FWD_REV_CMD_SIGN * curFwdRevCmd * power * -1;
                    curSideToSideCmd = Constants.SWERVE_SIDE_TO_SIDE_CMD_SIGN * curSideToSideCmd * power;
                }
            }
            if (isPicker) {
                armFinalCmd = -MathUtil.applyDeadband((Constants.SWERVE_FWD_REV_CMD_SIGN * driverController.getLeftY()), stickDeadband.get());
                armWithSpeed = Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC * armFinalCmd;
            }
            if (isDriver) {
                double objalignrange = driverController.getRightTriggerAxis();
                if (objalignrange > 0.50) {
                    doAutoObjectAlign = true;
                } else {
                    doAutoObjectAlign = false;
                }
            }
            curFwdRevCmd = MathUtil.applyDeadband(curFwdRevCmd, stickDeadband.get()) * translateCmdScalar.get();
            curRotCmd = MathUtil.applyDeadband(curRotCmd, stickDeadband.get()) * rotateCmdScalar.get();
            curSideToSideCmd = MathUtil.applyDeadband(curSideToSideCmd, stickDeadband.get()) * translateCmdScalar.get();
            if (isDriver) {
                if (driverController.getLeftStickButton()) {
                    curFwdRevCmd = curFwdRevCmd / 2.0;
                    curSideToSideCmd = curSideToSideCmd / 2.0;
                } else if (driverController.getRightStickButton()) {
                    curRotCmd = curRotCmd / 2.0;
                }
            } else {
                curFwdRevCmd = curFwdRevCmd / 2.0;
                curSideToSideCmd = curSideToSideCmd / 2.0;
                curRotCmd = curRotCmd / 2.0;
            }
            double limit_joystick_scale_factor = 1.0;
            fwdRevSlewCmd = fwdRevSlewLimiter.calculate(curFwdRevCmd) * limit_joystick_scale_factor;
            rotSlewCmd = rotSlewLimiter.calculate(curRotCmd) * limit_joystick_scale_factor;
            sideToSideSlewCmd = sideToSideSlewLimiter.calculate(curSideToSideCmd) * limit_joystick_scale_factor;
            if (isDriver) {
                robotRelative = driverController.getRightBumper();
            }
            alignRobotDir = isDriver;
            clawOpen = false;
            clawCone = false;
            clawCube = false;
            clawStop = false;
            presetU = false;
            presetR = false;
            presetD = false;
            presetL = false;
            armUp = false;
            armDown = false;
            armFront = false;
            armBack = false;
            matchedBasically = false;
            currentAngle = PoseTelemetry.getInstance().estimatedPose.getRotation().getDegrees() % 90;
            if (currentAngle < 0.99 && currentAngle > -0.99) {
                matchedBasically = true;
            }
            if (alignRobotDir == false) {
                if (driverController.getLeftBumper() == false) {
                    clawOpen = driverController.getAButton();
                    clawCube = driverController.getXButton();
                    clawCone = driverController.getYButton();
                    clawStop = driverController.getBButton();
                } else if (driverController.getLeftBumper() == true) {
                    armDown = driverController.getAButton();
                    armFront = driverController.getXButton();
                    armUp = driverController.getYButton(); 
                    armBack = driverController.getBButton();
                }
            } else if (alignRobotDir == true) {
                presetU = driverController.getYButton();
                presetR = driverController.getBButton();
                presetD = driverController.getAButton();
                presetL = driverController.getXButton();
            }
            changeToPreset = false;
            if (presetD == true || presetL == true || presetR == true || presetU == true) {
                changeToPreset = true;
            }
            if (presetU == true) {
                changeTo = 0.00;
            }
            if (presetL == true) {
                changeTo = 90;
            }
            if (presetR == true) {
                changeTo = 270.00;
            }
            if (presetD == true) {
                changeTo = 180.00;
            }
            if (changeToPreset) {
                if (Robot.isReal()) {
                    currentAngle = PoseTelemetry.getInstance().estimatedPose.getRotation().getDegrees();
                } else {
                    currentAngle = PoseTelemetry.getInstance().estimatedPose.getRotation().getDegrees();
                }
                if (currentAngle < 0) {
                    currentAngle = 360 + currentAngle;
                }
                if (changeTo > currentAngle) {
                    methOne = changeTo - currentAngle;
                    methTwo = "left";
                } else {
                    methOne = currentAngle - changeTo;
                    methTwo = "right";
                }
                if (methOne > 180) {
                    methOne = 360 - methOne;
                    if (methTwo == "left") {
                        methTwo = "right";
                    } else {
                        methTwo = "left";
                    }
                }
                if (methOne > 90) {
                    speedFactor = 0.9;
                } 
                else if (methOne > 45) {
                    speedFactor = 0.7;
                } 
                else if (methOne > 30) {
                    speedFactor = 0.5;
                } 
                else if (methOne > 15) {
                    speedFactor = 0.4;
                } 
                else if (methOne > 0) {
                    speedFactor = 0.25;
                }
                if (methOne < 0.99) {
                    matchedBasically = true;
                    methTwo = "";
                }
                if (speedFactor < 0.001) {
                    methTwo = "";
                }
                curFwdRevCmd = 0;
                curSideToSideCmd = 0;
                if (methTwo == "left") {
                    curRotCmd = Constants.SWERVE_ROT_CMD_SIGN * speedFactor * -1;
                } 
                else if (methTwo == "right") {
                    curRotCmd = Constants.SWERVE_ROT_CMD_SIGN * speedFactor;
                }
                curFwdRevCmd = MathUtil.applyDeadband(curFwdRevCmd, stickDeadband.get()) * translateCmdScalar.get();
                curSideToSideCmd = MathUtil.applyDeadband(curSideToSideCmd, stickDeadband.get()) * translateCmdScalar.get();
                curRotCmd = MathUtil.applyDeadband(curRotCmd, stickDeadband.get()) * rotateCmdScalar.get();
                fwdRevSlewCmd = fwdRevSlewLimiter.calculate(curFwdRevCmd) * limit_joystick_scale_factor;
                rotSlewCmd = curRotCmd;
                sideToSideSlewCmd = sideToSideSlewLimiter.calculate(curSideToSideCmd) * limit_joystick_scale_factor;
            }
        } else {
            curFwdRevCmd = 0.0;
            curRotCmd = 0.0;
            curSideToSideCmd = 0.0;
            robotRelative = false;
            resetOdometry = false;
            clawOpen = false;
            clawCone = false;
            clawCube = false;
            clawStop = false;
            presetU = false;
            presetR = false;
            presetD = false;
            presetL = false;
            armDown = false;
            armUp = false;
            armFront = false;
            armBack = false;
        }
        if (fwdRevSlewRate.isChanged() ||
            rotSlewRate.isChanged() ||
            sideToSideSlewRate.isChanged()) {
            fwdRevSlewRate.acknowledgeValUpdate();
            rotSlewRate.acknowledgeValUpdate();
            sideToSideSlewRate.acknowledgeValUpdate();
            fwdRevSlewLimiter = new SlewRateLimiter(fwdRevSlewRate.get());
            rotSlewLimiter = new SlewRateLimiter(rotSlewRate.get());
            sideToSideSlewLimiter = new SlewRateLimiter(sideToSideSlewRate.get());
        }
    }

    public double getFwdRevCmd_mps() {
        return fwdRevSlewCmd * Constants.MAX_FWD_REV_SPEED_MPS;
    }

    public double getRotateCmd_radpersec() {
        return rotSlewCmd * Constants.MAX_ROTATE_SPEED_RAD_PER_SEC;
    }

    public double getSideToSideCmd_mps() {
        return sideToSideSlewCmd * Constants.MAX_FWD_REV_SPEED_MPS;
    }

    public double getArmSpeed_radpersec() {
        return armWithSpeed;
    }

    public boolean getRobotRelative() {
        return robotRelative;
    }

    public boolean getOdoResetCmd() {
        return resetOdometry;
    }

    public boolean getClawOpen() {
        return clawOpen;
    }

    public boolean getClawCube() {
        return clawCube;
    }

    public boolean getClawCone() {
        return clawCone;
    }

    public boolean getClawStop() {
        return clawStop;
    }

    public boolean getArmUp() {
        return armUp;
    }

    public boolean getArmDown() {
        return armDown;
    }

    public boolean getArmFront() {
        return armFront;
    }

    public boolean getArmBack() {
        return armBack;
    }

    public boolean getPresetU() {
        return presetU;
    }

    public boolean getPresetD() {
        return presetD;
    }
    public boolean getPresetL() {
        return presetL;
    }

    public boolean getPresetR() {
        return presetR;
    }

    public boolean getSpinMoveCmd() {
        return spinMoveCmd;
    }

    public boolean getdoAutoObjectAlign() {
        return doAutoObjectAlign;
    }

    public boolean getDriveToCenterCmd() {
        return driveToCenterCmd;
    }

    public boolean getRemoveSoftLimits() {
        return stopSoftLimits;
    }
}