package frc.robot.Arm;
import frc.Constants;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Util.FunctionGenerator;
import frc.robot.Robot;
import edu.wpi.first.math.util.Units;


import frc.hardwareWrappers.SwerveAzmthEncoder.WrapperedSwerveAzmthEncoder;
import frc.hardwareWrappers.SwerveAzmthEncoder.WrapperedSwerveAzmthEncoder.SwerveAzmthEncType;

public class ArmPivotControl {
    
    /* Singleton infrastructure */
    private static ArmPivotControl instance;
    public static ArmPivotControl getInstance() {
        if (instance == null) {
            instance = new ArmPivotControl();
        }
        return instance;
    }

    // The Arm Pivot Joint Motors under our control
    // Naming assumes you are looking down on the top of the robot
    // Don't use "Rear", since both "Rear" and "Right" start with the same letter
    // F = Front
    // B = Back
    //
    // We removed the Front
    ArmMotorControl moduleB;

    double stopped_pos_rotations=-10;
    double current_pos_rotations=-10;
    double currentVel = 0;
    double scalar_correction_multiple_up=0.7; // was 0.5, 0.75 of 2.5 //1.8 needs to be smaller given that spring is assisting in up --> +0.4 -> a little faster --> +0.4
    double scalar_correction_multiple_down=-0.5; //0.5 of -0. //-0.2 needs more larger mag since spring is pulling back up --> -0.5 -> terrible bounce --> -0.5
    //double scalar_correction_up_90;
    double chosen_velocity_up = 0;
    double chosen_velocity_down = 0;

    boolean hasRelativeFixed = false;

    @Signal
    public double offsetRelToAbs_deg = 0;
    @Signal 
    public double absoluteAngleArm_deg = 0;
    @Signal 
    public double absAngleFromRelativeEncoder_deg = 0;

    // Common PID Calibrations for modules
    Calibration moduleArmPivot_kP;
    Calibration moduleArmPivot_kI;
    Calibration moduleArmPivot_kD;
    Calibration moduleArmPivot_kV; // Feed-forward Voltage Constat - IE, how many volts to get a certain amount of motor speed in radians per second?
    Calibration moduleArmPivot_kS; // Feed-forward Static Constnat - IE, how many volts to overcome static friction and get any motion at all?
   
    WrapperedSwerveAzmthEncoder pivot_enc;

    private ArmPivotControl(){
        pivot_enc = new WrapperedSwerveAzmthEncoder(SwerveAzmthEncType.RevThroughBoreEncoder, "encoderVA", Constants.ARM_VERTICAL_IDX, Constants.ARM_VERTICAL_IDX_OFFSET_RAD);

        moduleArmPivot_kP = new Calibration("Arm Pivot Motor kP", "", 0.0015); 
        moduleArmPivot_kI = new Calibration("Arm Pivot Motor kI", "", 0.0);
        moduleArmPivot_kD = new Calibration("Arm Pivot Motor kD", "", 0.0);
        moduleArmPivot_kV = new Calibration("Arm Pivot Motor kV", "volts/radPerSec", 0.018);
        moduleArmPivot_kS = new Calibration("Arm Pivot Motor kS", "volts", 0.172);

        moduleB = new ArmMotorControl("B", Constants.B_ARM_MOTOR_CANID, Constants.INVERT_ARM_DIRECTION);

        calUpdate(true);

    }

   
    // Pass the current calibration values downard into child classes.
    // Should generally only be called during disabled, since we don't usually
    // want to change PID gains while running.
    public void calUpdate(boolean force){

        // guard these Cal updates with isChanged because they write to motor controlelrs
        // and that soaks up can bus bandwidth, which we don't want
        //There's probably a better way to do this than this utter horrible block of characters. But meh.
        // Did you know that in vsCode you can edit multiple lines at once by holding alt, shift, and then clicking and dragging?
        if(moduleArmPivot_kP.isChanged() ||
           moduleArmPivot_kI.isChanged() ||
           moduleArmPivot_kD.isChanged() ||
           moduleArmPivot_kV.isChanged() ||
           moduleArmPivot_kS.isChanged() || force){
            moduleB.setClosedLoopGains(moduleArmPivot_kP.get(), moduleArmPivot_kI.get(), moduleArmPivot_kD.get(), moduleArmPivot_kV.get(), moduleArmPivot_kS.get());
            moduleArmPivot_kP.acknowledgeValUpdate();
            moduleArmPivot_kI.acknowledgeValUpdate();
            moduleArmPivot_kD.acknowledgeValUpdate();
            moduleArmPivot_kV.acknowledgeValUpdate();
            moduleArmPivot_kS.acknowledgeValUpdate();
        }
    }

    // Cause all non-annotated signals to broadcast a new value for the loop.
    public void updateTelemetry(){
        //double sampleTime = Robot.loopStartTime;
        moduleB.updateTelemetry();
    }


    @Signal(units = "rps")
    public double curCmdSpeed_radpersec;
    
    public enum AutoArmPivotCtrl{
        MOVE_UP(0),
        MOVE_DOWN(1),
        STOP_MOVEMT(2);
    
        public final int armstate;
    
        private AutoArmPivotCtrl(int armstate) {
            this.armstate = armstate;
        }
    
        public int toInt() {
            return this.armstate;
        }
    }

    public enum AutoPivotState{ 
        MATCHING(0),
        MOVING_UP(1),
        REACHED_UP1(2),
        MOVING_DOWN(4),
        MOVEMENT_STOP1(6),
        MOVEMENT_STOP2(7),
        BOOTING(8);

		public final int armstate;

		private AutoPivotState(int armstate) {
			this.armstate = armstate;
		}

		public int toInt() {
			return this.armstate;
		}
	}

    AutoArmPivotCtrl curCmd = AutoArmPivotCtrl.STOP_MOVEMT;
    
    @Signal (units="int")
    public int forDashboardCurrentState = AutoArmPivotCtrl.STOP_MOVEMT.toInt();

    AutoPivotState curState = AutoPivotState.BOOTING;

    AutoPivotState prevState = AutoPivotState.BOOTING;

    public void setArm(AutoArmPivotCtrl cmd){
        curCmd = cmd;
    }

    public void setArmSpeed(double cmdSpeed_radpersec){
        curCmdSpeed_radpersec = cmdSpeed_radpersec;
    }

    public void clearSoftLimits() {
        moduleB.setSoftLimits((float)0, (float)0, false);
    }

    public void update() { 

        pivot_enc.update();
        absoluteAngleArm_deg = Units.radiansToDegrees(pivot_enc.getAngle_rad());
        absAngleFromRelativeEncoder_deg = Units.radiansToDegrees(moduleB.armMotorCtrl.getPosition_rad())/Constants.MAGIC_ARM_RATIO + offsetRelToAbs_deg;

        //System.out.println(moduleB.armMotorCtrl.getAppliedVoltage_V());

        switch (prevState) {
            case BOOTING:
            case MATCHING:
                    curState = AutoPivotState.REACHED_UP1;
                break;
            case MOVING_UP:
            case MOVING_DOWN:
                switch (curCmd) {
                    case MOVE_UP:
                        curState = AutoPivotState.MOVING_UP;
                        break;
                    case MOVE_DOWN:
                        curState = AutoPivotState.MOVING_DOWN;
                        break;
                    case STOP_MOVEMT:
                        curState = AutoPivotState.MOVEMENT_STOP1;
                    default:
                        curState = AutoPivotState.MOVEMENT_STOP1;
                }
                break;
            case MOVEMENT_STOP2:
                switch (curCmd) {
                    case MOVE_UP:
                        curState = AutoPivotState.MOVING_UP;
                        break;
                    case MOVE_DOWN:
                        curState = AutoPivotState.MOVING_DOWN;
                        break;
                    case STOP_MOVEMT:
                    default:
                        curState = AutoPivotState.MOVEMENT_STOP2;
                }
                break;
            case MOVEMENT_STOP1:
                curState = AutoPivotState.MOVEMENT_STOP2;
                break;
            case REACHED_UP1:
                curState = AutoPivotState.MOVEMENT_STOP1;
                moduleB.armMotorCtrl.resetDistance();
                {
                    offsetRelToAbs_deg = absoluteAngleArm_deg;

                    double fwdLimit_deg = Constants.FWD_ARM_LIMIT_DEG-offsetRelToAbs_deg;   // if we start out at abs -10 then rel 0 is abs -10 the fwd limit should move from -1 to -9.
                    double fwdLimit_revolutions = fwdLimit_deg / 360.0;
                    double fwdLimit_unwrapped_revolutions = fwdLimit_revolutions*Constants.MAGIC_ARM_RATIO;
                    double revLimit_deg = Constants.REV_ARM_LIMIT_DEG-offsetRelToAbs_deg; // if we start out at abs -10 then rel 0 is abs -10 the rev limit should move from -130 to -120.
                    double revLimit_revolutions = revLimit_deg / 360.0;
                    double revLimit_unwrapped_revolutions = revLimit_revolutions*Constants.MAGIC_ARM_RATIO;
    
                    moduleB.setSoftLimits((float)fwdLimit_unwrapped_revolutions, (float)revLimit_unwrapped_revolutions, true); // Sets the distance to 0 radians.
                }

                break;
            default:
                curState = AutoPivotState.MOVEMENT_STOP1;
        }

        forDashboardCurrentState = curState.toInt();

        //System.out.println("new arm state:"+curState.name());
        int currentLimit_A = 10;
        switch (curState) {
            case BOOTING:
            case MATCHING:
            break;
            case MOVING_UP:
            case MOVING_DOWN:
                double raw_pos_rad = moduleB.armMotorCtrl.getPosition_rad()+Units.degreesToRadians(offsetRelToAbs_deg);
                double raw_pos_revolutions = raw_pos_rad/Math.PI/2.0;
                double raw_pos_gear_reduced_revolutions = raw_pos_revolutions / Constants.MAGIC_ARM_RATIO;
                double raw_pos_degrees = raw_pos_gear_reduced_revolutions * 360.0;
                moduleB.setDesiredState(curCmdSpeed_radpersec, currentLimit_A);
                moduleB.update(true);
                break;

            case MOVEMENT_STOP1:
                stopped_pos_rotations = Units.degreesToRotations(Units.radiansToDegrees(moduleB.armMotorCtrl.getPosition_rad())+offsetRelToAbs_deg);
                moduleB.setDesiredState(0, currentLimit_A);
                break;

            case MOVEMENT_STOP2:

                // stopped=0 current_pos_rotations -10  
                // 0--10 = +10 = difference
                // we want to go up
                current_pos_rotations = Units.degreesToRotations(Units.radiansToDegrees(moduleB.armMotorCtrl.getPosition_rad())+offsetRelToAbs_deg);
                double difference = stopped_pos_rotations-current_pos_rotations;
                currentVel = 0;
                if (difference>=0) {
                    chosen_velocity_up = scalar_correction_multiple_up*Constants.MAGIC_ARM_RATIO;
                    currentVel = chosen_velocity_up;
                    chosen_velocity_down = 0;
                }
                else {
                    chosen_velocity_down = scalar_correction_multiple_down*Constants.MAGIC_ARM_RATIO;
                    currentVel = chosen_velocity_down;
                    chosen_velocity_up = 0;
                }
                moduleB.setDesiredState(currentVel, currentLimit_A);
                moduleB.update(true);
                break;

            case REACHED_UP1:
            default:
                moduleB.setDesiredState(0, currentLimit_A);
                moduleB.update(true);
                break;
        }
            

        // Update previous 
        prevState = curState;
        //System.out.println("new arm state:"+curState.name());
        //System.out.println("Stopped angle: "+stopped_pos_rotations);
        //System.out.println("Current angle: "+current_pos_rotations);
        //System.out.println("ActualMove curCmdSpeed_radpersec"+curCmdSpeed_radpersec+" "+currentVel);
    }
  

}

