package frc.robot;

import frc.robot.AutoDrive.AutoDrive;
import frc.Constants;
import frc.hardwareWrappers.Gyro.WrapperedGyro;
import frc.lib.Signal.Annotations.Signal;

public class ChargeStationBalancer {
  
  public static final double PITCH_THRESHOLD = 2.0; //deg
  public static final double ROLL_THRESHOLD = 2.0; //deg
  public static final double PITCH_KP = (0.02/3)*0.51; //proportional gain for pitch control
  public static final double ROLL_KP = 0.01; //proportional gain for roll control
  
  public WrapperedGyro gyro;

  AutoDrive ad;
  
  public ChargeStationBalancer(WrapperedGyro givenGyro) {
    gyro = givenGyro;

    ad = AutoDrive.getInstance();
  }

  @Signal
  double pitchError_deg = 0;
  @Signal
  double pitchSpeed = 0;

  public boolean balanceOnChargeStation() {
    System.out.println("pitch speed: "+pitchSpeed);
    boolean balanced = isBalanced();
    if (!balanced) {
      //calculate the error in pitch and roll angles
      pitchError_deg = getPitchAngle_deg() - 0.0; // 0.0 is the target pitch angle
      double rollError_deg = getRollAngle_deg() - 0.0; // 0.0 is the target roll angle
      
      // calculate the motor speeds based on the pitch and roll errors
      
      pitchSpeed = pitchError_deg * PITCH_KP;
      double rollSpeed = rollError_deg * ROLL_KP;
      
      // adjust the motor speeds to keep the robot balanced
      
      double forwd = 0;
      double right = 0;
      if (Math.abs(pitchError_deg) > PITCH_THRESHOLD) {
        forwd = pitchSpeed;
        // 11-15 degrees (start vs end)
        // (negative included depending on if - is for up or down pitch and correction should be f/b...)
      }
      ad.transformRobot(forwd,right,0,true);
    }
    else {
      ad.transformRobot(0,0,0,true);
    }
    return balanced;
  }
  
  public boolean isBalanced() {
    // check if the robot is within the pitch and roll thresholds for balancing
    double pitchAngle = getPitchAngle_deg();
    //double rollAngle = getRollAngle_deg();
    //return Math.abs(pitchAngle) < PITCH_THRESHOLD && Math.abs(rollAngle) < ROLL_THRESHOLD;
    return Math.abs(pitchAngle) < PITCH_THRESHOLD;
  }

  @Signal
  double prevPitch = 0;
  @Signal
  double curPitch = 0;
  
  public double getPitchAngle_deg() {
    curPitch = -gyro.getRoll_deg();
    if (Math.abs(curPitch-prevPitch)>5) {
      return prevPitch;
    }
    else {
      prevPitch = curPitch;
      return curPitch;
    }
  }
  
  public double getRollAngle_deg() {
    return gyro.getPitch_deg();
  }
}