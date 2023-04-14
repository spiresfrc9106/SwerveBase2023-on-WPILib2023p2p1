package frc.robot;
import frc.lib.Signal.SignalUtils;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Webserver2.Webserver2;
import frc.lib.Webserver2.DashboardConfig.DashboardConfig;
import frc.lib.Webserver2.DashboardConfig.SwerveStateTopicSet;
import frc.robot.Arm.ClawControl;

public class Dashboard {
    @Signal(name = "db_open")
    boolean openClawState;
    @Signal(name = "db_cube")
    boolean cubeClawState;
    @Signal(name = "db_cone")
    boolean coneClawState;
    @Signal(name = "db_stopc")
    boolean stopClawState;
    @Signal(name = "db_openlim")
    boolean openLimState;
    @Signal(name = "db_closelim")
    boolean closeLimState;
    @Signal(name = "db_visionTargetAngle")
    double visionTargetAngle;
    @Signal(name = "db_visionTargetVisible")
    boolean visionTargetVisible;
    @Signal(name = "db_masterCaution")
    boolean masterCaution;
    String masterCautionTxt;
    @Signal(name = "db_shooterSpeed")
    double shooterSpeed;
    @Signal(name = "db_pneumaticsPressure")
    double pnuemPressure;
    @Signal(name = "db_shooterSpoolup")
    boolean shooterSpoolup;
    @Signal(name = "db_hopperFull")
    boolean hopperFull;
    @Signal(name = "db_Yeet")
    boolean Yeet;
    @Signal(name = "db_clmberExtend")
    boolean climberExtend;
    @Signal(name = "db_fbctrl")
    double fbctrl;
    @Signal(name = "db_lrctrl")
    double lrctrl;
    @Signal(name = "db_fblrc")
    double fblrc;
    @Signal(name = "db_rotcmd")
    double rotcmd;
    @Signal(name = "db_presetU")
    boolean dpresetU;
    @Signal(name = "db_presetD")
    boolean dpresetD;
    @Signal(name = "db_presetL")
    boolean dpresetL;
    @Signal(name = "db_presetR")
    boolean dpresetR;
    @Signal(name = "db_preset")
    boolean presetNone;
    @Signal(name = "db_peace")
    boolean isAligned;
    @Signal(name = "db_armpvt")
    double armspeed;
    @Signal(name = "db_armdirectl")
    boolean armdirectionl;
    @Signal(name = "db_armdirectr")
    boolean armdirectionr;
    @Signal(name = "db_bumpone")
    boolean bumpone;
    @Signal(name = "db_bumptwo")
    boolean bumptwo;
    @Signal(name = "db_bumpthree")
    boolean bumpthree;
    @Signal(name = "db_rightst")
    boolean rightst;
    @Signal(name = "db_leftst")
    boolean leftst;
    boolean pneumaticPressureLow = false; //TODO?
    DashboardConfig d;

    public Dashboard(Webserver2 ws_in) {
        d = ws_in.dashboard;
        final double LEFT_COL = 17;
        final double CENTER_COL = 50;
        final double RIGHT_COL = 83;
        final double ROW1 = 15;
        final double ROW2 = 46;
        final double ROW3 = 58; //55
        final double ROW4 = 70; //68
        d.addCamera("cam1", "http://10.17.36.10:1181/stream.mjpg", LEFT_COL - 1, ROW2 + 3, 0.7);
        d.addFieldPose("pose", "Field", LEFT_COL, ROW1, 0.75);
        SwerveStateTopicSet[] topicList = new SwerveStateTopicSet[4];
        topicList[0] = new SwerveStateTopicSet("FL", 0);
        topicList[1] = new SwerveStateTopicSet("FR", 1);
        topicList[2] = new SwerveStateTopicSet("BL", 2);
        topicList[3] = new SwerveStateTopicSet("BR", 3);
        d.addSwerveState(topicList, "SwerveState", CENTER_COL - 8, ROW2 + 7, 0.6); //rt + 3.5
        d.addCircularGauge(SignalUtils.nameToNT4ValueTopic("db_fblrc"), "Translation", "mps", 0, 4.4831, 0, 4.4831, CENTER_COL - 7, ROW1, 1.0);
        d.addCircularGauge(SignalUtils.nameToNT4ValueTopic("db_rotcmd"), "Rotation", "radpersec", 0, 2.5135, 0, 2.5135, CENTER_COL + 13, ROW1, 1.0);
        d.addCircularGauge(SignalUtils.nameToNT4ValueTopic("db_armpvt"), "Arm Pivot", "radpersec", 0, 12.345, 0, 12.345, CENTER_COL + 33, ROW1, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_masterCaution"), "Master Caution", "#FF0000", "icons/alert.svg", CENTER_COL + 6, ROW2, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_visionTargetVisible"), "Vision Target Visible", "#00FF00", "icons/vision.svg", CENTER_COL + 12, ROW2, 1.0);
        d.addSound(SignalUtils.nameToNT4ValueTopic("db_Yeet"), "YEET", "sfx/YEET.mp3", false);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_leftst"), "Translation Slowed", "#ff7300", "icons/ts.svg", CENTER_COL + 18, ROW2, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_rightst"), "Rotation Slowed", "#a020f0", "icons/rs.svg", CENTER_COL + 24, ROW2, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_armdirectl"), "Arm Up", "#ffd700", "icons/aup.svg", CENTER_COL + 30, ROW2, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_armdirectr"), "Arm Down", "#3C92E6", "icons/adown.svg", CENTER_COL + 36, ROW2, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_bumptwo"), "Robot-Orientated", "#ffffff", "icons/robo.svg", CENTER_COL + 42, ROW2, 1.0);
        openClawState = false;
        cubeClawState = false;
        coneClawState = false;
        stopClawState = false;
        closeLimState = false;
        openLimState = false;
        if (ClawControl.getInstance().forDashboardCurrentState == 0) {
            openClawState = true;
        }
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_open"), "Claw Opening", "#00ff00", "icons/open.svg", CENTER_COL + 12, ROW3, 1.0);

        if (ClawControl.getInstance().forDashboardCurrentState == 1) {
            cubeClawState = true;
        }
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_cube"), "Claw Closing on Cube", "#3C92E6", "icons/cube.svg", CENTER_COL + 24, ROW3, 1.0);

        if (ClawControl.getInstance().forDashboardCurrentState == 2) {
            coneClawState = true;
        }
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_cone"), "Claw Closing on Cone", "#ffd700", "icons/cone.svg", CENTER_COL + 30, ROW3, 1.0);

        if (ClawControl.getInstance().forDashboardCurrentState == 3) {
            stopClawState = true;
        }
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_stopc"), "Claw Stopped", "#ff0000", "icons/cstop.svg", CENTER_COL + 6, ROW3, 1.0);

        if (ClawControl.getInstance().forDashboardCurrentState == 4) {
            closeLimState = true;
        }
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_openlim"), "Claw Opened", "#a020f0", "icons/openlim.svg", CENTER_COL + 18, ROW3, 1.0);

        if (ClawControl.getInstance().forDashboardCurrentState == 5) {
            openLimState = true;
        }
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_closelim"), "Claw Closed", "#ff7300", "icons/closelim.svg", CENTER_COL + 36, ROW3, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_bumpthree"), "Commands Available", "#ffffff", "icons/1st.svg", CENTER_COL + 42, ROW3, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_preset"), "No Assisted Turns", "#ff7300", "icons/free.svg", CENTER_COL + 6, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_presetU"), "Turning Up", "#ffd700", "icons/up.svg", CENTER_COL + 12, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_presetD"), "TUrning Down", "#00ff00", "icons/down.svg", CENTER_COL + 24, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_presetL"), "Turning Left", "#3C92E6", "icons/left.svg", CENTER_COL + 30, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_presetR"), "Turning Right", "#ff0000", "icons/right.svg", CENTER_COL + 18, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_peace"), "Perfectly Aligned", "#a020f0", "icons/none.svg", CENTER_COL + 36, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_bumpone"), "Commands Available", "#ffffff", "icons/2nd.svg", CENTER_COL + 42, ROW4, 1.0);
    }

    public void updateDriverView() {
        if (pnuemPressure < 80.0) {
            masterCautionTxt = "Low Pneumatic Pressure";
            masterCaution = false;
        } else {
            masterCautionTxt = "";
            masterCaution = false;
        }
    }
}