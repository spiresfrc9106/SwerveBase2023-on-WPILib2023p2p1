// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.Constants;
import frc.lib.Calibration.CalWrangler;
import frc.lib.LoadMon.RIOLoadMonitor;
import frc.lib.LoadMon.SegmentTimeTracker;
import frc.lib.Signal.SignalWrangler;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Webserver2.Webserver2;
import frc.robot.AutoDrive.AutoDrive;
import frc.robot.AutoDrive.AutoDrive.AutoDriveCmdState;
import frc.robot.Autonomous.AutoPickupSystem;
import frc.robot.Autonomous.Autonomous;
import frc.robot.Drivetrain.DrivetrainControl;
import frc.sim.RobotModel;
import frc.robot.Arm.ArmPivotControl;
import frc.robot.Arm.ClawControl;
import frc.robot.Arm.ArmPivotControl.AutoArmPivotCtrl;
import frc.robot.Arm.ClawControl.AutoClawFingerCtrl;
//NEW
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private boolean enableCameras = false;

  public static double loopStartTime;

  ///////////////////////////////////////////////////////////////////
  // Instatntiate new classes after here 
  // ...


  // Website utilities
  Webserver2 webserver;
  Dashboard db;
  CalWrangler cw;

  // Things
  RIOLoadMonitor loadMon;
  BatteryMonitor batMan;

  // Main Driver
  DriverInput di;
  DriverInput pi;

  //Drivetrain and drivetrain accessories
  DrivetrainControl dt;
  AutoDrive ad;
  ClawControl fng;
  ArmPivotControl armPvt;

  // Autonomous Control Utilities
  Autonomous auto;
  PoseTelemetry pt;

  AutoPickupSystem autoalign;

  // Cameras

  CenterCamera cc;

  SegmentTimeTracker stt;

  @Signal(units = "sec")
  double mainLoopDuration;
  @Signal(units = "sec")
  double mainLoopPeriod;

  @Signal(units = "in")
  double sonarDistance_in = 0;

  public double arm_speed_radpersec;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  private final AnalogInput ultrasonic = new AnalogInput(0);


  // ... 
  // But before here
  ///////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////
  // Do one-time initilization here
  ///////////////////////////////////////////////////////////////////


  @Override
  public void robotInit() {

    stt = new SegmentTimeTracker("Robot.java", 0.02);

    stt.start();
	
	Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      // Running on a real robot, log to a USB stick
      case REAL:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        logger.addDataReceiver(new NT4Publisher());
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        logger.addDataReceiver(new WPILOGWriter(""));
        logger.addDataReceiver(new NT4Publisher());
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()

    // Start AdvantageKit logger
    logger.start();

    // Disable default behavior of the live-window output manipulation logic
    // We've got our own and never use this anyway.
    LiveWindow.setEnabled(false);
    LiveWindow.disableAllTelemetry();
    stt.mark("LW Disable");

    NetworkTableInstance.getDefault().startServer();
    stt.mark("NT4");


    /* Init website utilties */
    webserver = new Webserver2();
    stt.mark("Webserver2");

    cw = CalWrangler.getInstance();
    stt.mark("Cal Wrangler");

    loadMon = new RIOLoadMonitor();
    stt.mark("RIO Load Monitor");

    //yavinNote: uncomment to debug the battery monitor; why is the battery monitor crashing?
    //batMan = BatteryMonitor.getInstance();

    stt.mark("Battery Monitor");

    //bcd = new Ballcolordetector();
    stt.mark("Ball Color Detector");

    di = new DriverInput(0);
    pi = new DriverInput(1);
    stt.mark("Driver IO");

    dt = DrivetrainControl.getInstance();
    ad = AutoDrive.getInstance();
    stt.mark("Drivetrain Control");

    fng = ClawControl.getInstance();
    stt.mark("Claw Control");

    armPvt = ArmPivotControl.getInstance();
    stt.mark("Arm Pivot Control");

    auto = Autonomous.getInstance();
    //auto.loadSequencer();
    stt.mark("Autonomous");

    autoalign = AutoPickupSystem.getInstance();

    pt = PoseTelemetry.getInstance();
    stt.mark("Pose Telemetry");

    if (enableCameras) {
      cc = CenterCamera.getInstance();
      stt.mark("Cameras");
    }


    db = new Dashboard(webserver);
    stt.mark("Dashboard");

    if(Robot.isSimulation()){
      simulationSetup();
    }
    syncSimPoseToEstimate();
    stt.mark("Simulation");

    SignalWrangler.getInstance().registerSignals(this);
    stt.mark("Signal Registration");

    webserver.startServer();
    stt.mark("Webserver Startup");

    // todo - Remove or change for our camera system.
    PhotonCamera.setVersionCheckEnabled(false);
    stt.mark("Photonvision Config");

    System.gc();
    stt.mark("Post Init GC");

    System.out.println("Init Stats:");
    stt.end();

  }


  ///////////////////////////////////////////////////////////////////
  // Autonomous-Specific
  ///////////////////////////////////////////////////////////////////
  @Override
  public void autonomousInit() {
    //System.out.println("auto-init ran");
    SignalWrangler.getInstance().logger.startLoggingAuto();
    //Reset sequencer
    auto.reset();
    //auto.startSequencer();

    auto.allowRun();
    // Ensure simulation resets to correct pose at the start of autonomous
    syncSimPoseToEstimate();
  }

  @Override
  public void autonomousPeriodic() {
    //System.out.println("auto-periodic ran");
    stt.start();
    loopStartTime = Timer.getFPGATimestamp();
    //Step the sequencer forward
    auto.update();
    stt.mark("Auto Update");
  }

  
  ///////////////////////////////////////////////////////////////////
  // Teleop-Specific
  ///////////////////////////////////////////////////////////////////
  @Override
  public void teleopInit() {
    auto.blockRun();
    SignalWrangler.getInstance().logger.startLoggingTeleop();
  }

  @Override
  public void teleopPeriodic() {
    stt.start();
    loopStartTime = Timer.getFPGATimestamp();

    di.update();
    pi.update();
    stt.mark("Driver Input");

    /////////////////////////////////////
    // Drivetrain Input Mapping

    boolean automaticSwerve = false;
    if (di.getdoAutoObjectAlign()) {
      automaticSwerve = true;
    }

    if(pi.getClawOpen()){
      fng.setFinger(AutoClawFingerCtrl.OPEN_PHALANGES);
    }
    else if (pi.getClawCube()){
      fng.setFinger(AutoClawFingerCtrl.CLOSE_FOR_CUBE);
    }
    else if (pi.getClawCone()) {
      fng.setFinger(AutoClawFingerCtrl.CLOSE_FOR_CONE);
    }
    else if (pi.getClawStop()) {
      fng.setFinger(AutoClawFingerCtrl.STOP_PHALANGES);
    }
    else {
      //YN: took this out to see if the command was stopping the claw from working
      //YN: if no new commands, the close and open limits will remain constant
      //fng.setFinger(AutoClawFingerCtrl.STOP_PHALANGES);
    }

    boolean armExistsAngle = false;
    double desiredArmAngle = -2;
    if(pi.getArmDown()){
      desiredArmAngle = -130;
      armExistsAngle = true;
    }
    else if (pi.getArmFront()){
      desiredArmAngle = -90;
      armExistsAngle = true;
    }
    else if (pi.getArmUp()) {
      desiredArmAngle = -60;
      armExistsAngle = true;
    }
    else if (pi.getArmBack()) {
      desiredArmAngle = -2;
      armExistsAngle = true;
    }
    
    if (armExistsAngle) {
      double angle = armPvt.absoluteAngleArm_deg;
      if (angle<(desiredArmAngle) && (desiredArmAngle-angle)>2) {
        //angle -90, desired -60
        // -60 --90 == 30, >2
        if ((desiredArmAngle-angle)>30) {
          armPvt.setArmSpeed(0.9*Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
        }
        else {
          armPvt.setArmSpeed(0.7*Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
        }
        armPvt.setArm(AutoArmPivotCtrl.MOVE_UP);
      }
      else if (angle>(desiredArmAngle) && (angle-(desiredArmAngle))>2) {
        //angle -30, desired -60
        // -30 --60 = 30
        if ((angle-(desiredArmAngle))>30) {
          armPvt.setArmSpeed(0.9*Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
        }
        else {
          armPvt.setArmSpeed(0.7*Constants.ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC);
        }
        armPvt.setArm(AutoArmPivotCtrl.MOVE_DOWN);
      } else {
          armPvt.setArmSpeed(0);
          armPvt.setArm(AutoArmPivotCtrl.STOP_MOVEMT);
      }
    }
    else {
      arm_speed_radpersec = pi.getArmSpeed_radpersec();

      armPvt.setArmSpeed(arm_speed_radpersec);
      if (arm_speed_radpersec<0) {
        armPvt.setArmSpeed(arm_speed_radpersec); //smaller accounting gravity
        armPvt.setArm(AutoArmPivotCtrl.MOVE_DOWN);
      }
      else if (arm_speed_radpersec>0) {
        armPvt.setArm(AutoArmPivotCtrl.MOVE_UP);
      }
      else {
        armPvt.setArm(AutoArmPivotCtrl.STOP_MOVEMT);
      }
    }



    if (pi.getRemoveSoftLimits() == true) {
      System.out.println("Disabled soft limits for Robot Arm");
    }
/*
    double d_fwrd = di.getFwdRevCmd_mps();
    double d_side = di.getSideToSideCmd_mps();
    double d_spin = di.getRotateCmd_radpersec();
    double p_fwrd = pi.getFwdRevCmd_mps();
    double p_side = pi.getSideToSideCmd_mps();
    double p_spin = pi.getRotateCmd_radpersec();*/
    boolean robor = di.getRobotRelative();
    double f_fwrd = di.getFwdRevCmd_mps();
    double f_side = di.getSideToSideCmd_mps();
    double f_spin = di.getRotateCmd_radpersec();/*
    if (robor==true) {
      //ignore up/down left/right cmd from pi
      f_fwrd = d_fwrd;
      f_side = d_side;
      //but... rotation can still work
      f_spin = d_spin+p_spin;
    }
    else {
      f_fwrd = d_fwrd + p_fwrd;
      f_side = d_side + p_side;
      f_spin = d_spin + p_spin;
    }*/
    if (automaticSwerve==false) {
      ad.setManualCommands(f_fwrd,f_side,f_spin,!robor);
    }
    else {
      autoalign.update(sonarDistance_in);
      ad.setManualCommands(autoalign.fwdVelocity_mps, 0, autoalign.azVelocity, false);
    }

    db.fbctrl = di.getFwdRevCmd_mps();
    db.lrctrl = di.getSideToSideCmd_mps();
    db.rotcmd = di.getRotateCmd_radpersec();
    db.fblrc = Math.sqrt((db.fbctrl*db.fbctrl)+(db.lrctrl*db.lrctrl));

    if (db.fblrc<0) {
      db.fblrc = db.fblrc*-1;
    }
    if (db.rotcmd<0) {
      db.rotcmd=db.rotcmd*-1;
    }

    db.dpresetU = di.presetU;
    db.dpresetD = di.presetD;
    db.dpresetL = di.presetL;
    db.dpresetR = di.presetR;

    if (di.presetU==false&&di.presetD==false&&di.presetR==false&&di.presetL==false) {
      db.presetNone = true;
    }
    else {
      db.presetNone = false;
    }

    db.isAligned = di.matchedBasically;
    if (di.getArmSpeed_radpersec()<0) {
    db.armspeed = di.getArmSpeed_radpersec()*-1;
    db.armdirectionl = false;  //YN: might need to switch this if directions are opp
    db.armdirectionr = true;
    }
    else if (di.getArmSpeed_radpersec()>1){
    db.armspeed = di.getArmSpeed_radpersec();
    db.armdirectionl = true;
    db.armdirectionr = false;
    }
    else {
      db.armspeed = di.getArmSpeed_radpersec();
      db.armdirectionl = false;
      db.armdirectionr = false; 
    }

    db.bumpone = di.alignRobotDir;
    db.bumptwo = di.robotRelative;
    db.rightst = di.rightStickButtonCode;
    db.leftst = di.leftStickButtonCode;

    if (di.alignRobotDir==false) {
      db.bumpthree = true;
    }
    else {
      db.bumpthree = false;
    }

    //ad.update();


    if(di.getOdoResetCmd()){
      //Reset pose estimate to angle 0, but at the same translation we're at
      Pose2d newPose = new Pose2d(dt.getCurEstPose().getTranslation(), new Rotation2d(0.0));
      dt.setKnownPose(newPose);
    }


    stt.mark("Human Input Mapping");

  }



  ///////////////////////////////////////////////////////////////////
  // Disabled-Specific
  ///////////////////////////////////////////////////////////////////
  @Override
  public void disabledInit() {
    SignalWrangler.getInstance().logger.stopLogging();
  }

  @Override
  public void disabledPeriodic() {
    stt.start();
    loopStartTime = Timer.getFPGATimestamp();

    
    dt.calUpdate(false);
    stt.mark("Cal Updates");

    auto.sampleDashboardSelector();
    stt.mark("Auto Mode Update");

  }



  
  ///////////////////////////////////////////////////////////////////
  // Common Periodic updates
  ///////////////////////////////////////////////////////////////////
  @Override
  public void robotPeriodic() {
    
    sonarDistance_in = getSonarDistance();

    if (enableCameras) {
      cc.update();
    }

    stt.mark("Camera Update");
    if(DriverStation.isTest() && !DriverStation.isDisabled()){
      dt.testUpdate();
      stt.mark("Drivetrain  ");
    } else {
      dt.update();
      stt.mark("Drivetrain  ");
      fng.update();
      stt.mark("Fingers     ");
      armPvt.update();
      stt.mark("Arm         ");
    }


    cw.update();
    stt.mark("Cal Wrangler");
    db.updateDriverView();
    stt.mark("Dashboard   ");
    telemetryUpdate();
    stt.mark("Telemetry   ");
    

    stt.end();

    ad.update();

  }

  private void telemetryUpdate(){
    double time = loopStartTime;

    dt.updateTelemetry();
    fng.updateTelemetry();
    armPvt.updateTelemetry();

    pt.setDesiredPose(dt.getCurDesiredPose());
    pt.setEstimatedPose(dt.getCurEstPose());
    
    pt.update(time);

    mainLoopDuration = stt.loopDurationSec;
    mainLoopPeriod = stt.loopPeriodSec;

    SignalWrangler.getInstance().sampleAllSignals(time);
  }

  ///////////////////////////////////////////////////////////////////
  // Test-Mode-Specific
  ///////////////////////////////////////////////////////////////////

  @Override
  public void testInit(){
    // Tell the subsystems that care that we're entering test mode.
    dt.testInit();
  }

  @Override
  public void testPeriodic(){
    stt.start();
    loopStartTime = Timer.getFPGATimestamp();


    // Nothing special here, yet
  }

  ///////////////////////////////////////////////////////////////////
  // Simulation Support
  ///////////////////////////////////////////////////////////////////

  RobotModel plant;

  public void simulationSetup(){
    plant = new RobotModel();
    syncSimPoseToEstimate();
  }

  public void syncSimPoseToEstimate(){
    if(Robot.isSimulation()){
      plant.reset(dt.getCurEstPose());
    }
  }

  @Override
  public void simulationPeriodic(){
    plant.update(this.isDisabled());
    pt.setActualPose(plant.getCurActPose());
  }


  public double getSonarDistance() {
    double rawValue = ultrasonic.getValue();
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    //double currentDistanceCentimeters = rawValue * voltage_scale_factor * 0.125;
    double currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;

    sonarDistance_in = currentDistanceInches;

    return currentDistanceInches;
  }
}
