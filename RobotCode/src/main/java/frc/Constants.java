package frc;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class Constants {
    //////////////////////////////////////////////////////////////////
    // Drivetrain Physical
    //////////////////////////////////////////////////////////////////
    static public final double HALF_WIDTH_M = Units.inchesToMeters(16.5 / 2.0);
    static public final double HALF_LENGTH_M = Units.inchesToMeters(26.5 / 2.0);
    static public final double WHEEL_BASE_HALF_WIDTH_M = Units.inchesToMeters(26.5 / 2.0); // todo - work towards removing this.
    static public final double ROBOT_MASS_kg = UnitUtils.lbsToKg(100); //YN: change 100 to ---?
    static public final double ROBOT_MOI_KGM2 = 1.0 / 12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_HALF_WIDTH_M * 2.2), 2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center

    // Drivetrain Performance Mechanical limits
    static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(19.0); //YAVIN-NOTE: 19/13x faster
    static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(19.0); //YAVIN-NOTE: 19/13x faster
    static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(180.0); //YAVIN-NOTE: 2x faster
    static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS / 1.00; //0-full time of 0.25 second
    static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC / .25; //0-full time of 0.25 second

    // See https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081
    static public final double WHEEL_GEAR_RATIO = 4.71; //L2 gearing
    static public final double AZMTH_GEAR_RATIO = 4.71; //todo - fix me up
    static public final double WHEEL_FUDGE_FACTOR = 0.9238; // carpet roughtop scrub factor
    static public final double WHEEL_RADIUS_IN = 3.0 / 2.0 * WHEEL_FUDGE_FACTOR; //THREE inch diameter wheels - https://www.swervedrivespecialties.com/collections/mk4i-parts/products/billet-wheel-4d-x-1-5w-bearing-bore


    static public final boolean azmth_motor_inverted() {
        if (Robot.isReal()) {
            return false;
        } else {
            return true;
        }
    }

    static public final boolean INVERT_AZMTH_MOTOR = azmth_motor_inverted();

    static public final boolean[] wheel_direction() {
        boolean[] INVERT_WHEEL_DIRECTION_ISREAL = {
            false,
            false,
            false,
            false
        };
        boolean[] INVERT_WHEEL_DIRECTION_ISSIM = {
            true,
            false,
            true,
            false
        };
        if (Robot.isReal()) {
            return INVERT_WHEEL_DIRECTION_ISREAL;
        } else {
            return INVERT_WHEEL_DIRECTION_ISSIM;
        }
    }

    // Mechanical mounting offsets of the encoder & magnet within the shaft
    // Must be updated whenever the module is reassembled
    // Procedure: 
    // 0 - Put the robot up on blocks.
    // 1 - Reset all these values to 0, deploy code
    // 2 - Pull up dashboard with encoder readings (in radians)
    // 3 - Using a square, twist the modules by hand until they are aligned with the robot's chassis
    // 4 - Read out the encoder readings for each module, put them here
    // 5 - Redeploy code, verify that hte encoder readings are correct as each module is manually rotated

    static public final double[] get_azimuth_encoder_mount_offset_rad() {
        double[] AZMTH_ENCODER_MOUNT_OFFSET_RAD_ISREAL = {
            Units.degreesToRadians(4.0 - 11.95 - 90.0),
            Units.degreesToRadians(41.89 - 90.0 + 180.0),
            Units.degreesToRadians(-1 + 35.5 - 90.0),
            Units.degreesToRadians(-3 - 10.5 - 90.0)
        };
        double[] AZMTH_ENCODER_MOUNT_OFFSET_RAD_ISSIM = {-2.157,
            -1.575,
            -2.180,
            -0.803
        };
        if (Robot.isReal()) {
            return AZMTH_ENCODER_MOUNT_OFFSET_RAD_ISREAL;
        } else {
            return AZMTH_ENCODER_MOUNT_OFFSET_RAD_ISSIM;
        }
    }

    static public final boolean[] INVERT_WHEEL_DIRECTION = wheel_direction();

    static public final double FL_ENCODER_MOUNT_OFFSET_RAD = get_azimuth_encoder_mount_offset_rad()[Constants.FL];
    static public final double FR_ENCODER_MOUNT_OFFSET_RAD = get_azimuth_encoder_mount_offset_rad()[Constants.FR];
    static public final double BL_ENCODER_MOUNT_OFFSET_RAD = get_azimuth_encoder_mount_offset_rad()[Constants.BL];
    static public final double BR_ENCODER_MOUNT_OFFSET_RAD = get_azimuth_encoder_mount_offset_rad()[Constants.BR];
    static public final double ARM_VERTICAL_IDX_OFFSET_RAD = Units.degreesToRadians(210); //150

    static public final double get_gyro_offset_rad() {
        if (Robot.isReal()) {
            return Units.degreesToRadians(180.0);
        } else {
            return 0.0;
        }
    }
    static public final double GYRO_OFFSET_RAD = get_gyro_offset_rad();

    // Location of vision cameras relative to robot center - currently front and back
    static public final Transform3d robotToFrontCameraTrans = new Transform3d(new Translation3d(WHEEL_BASE_HALF_WIDTH_M, 0, 1.0), new Rotation3d(0.0, 0.0, 0.0));
    static public final Transform3d robotToRearCameraTrans = new Transform3d(new Translation3d(-1.0 * WHEEL_BASE_HALF_WIDTH_M, 0, 1.0), new Rotation3d(0.0, 0.0, Math.PI));

    static public final boolean[] finger_direction() {
        boolean[] INVERT_FINGER_DIRECTION_ISREAL = {
            false,
            true
        };
        boolean[] INVERT_FINGER_DIRECTION_ISSIM = {
            false,
            true
        };
        if (Robot.isReal()) {
            return INVERT_FINGER_DIRECTION_ISREAL;
        } else {
            return INVERT_FINGER_DIRECTION_ISSIM;
        }
    }

    static public final boolean[] INVERT_FINGER_DIRECTION = finger_direction();

    static public final boolean INVERT_ARM_DIRECTION = false;

    //////////////////////////////////////////////////////////////////
    // Electrical
    //////////////////////////////////////////////////////////////////

    // PWM Bank
    //static public final int UNUSED = 0;
    //static public final int UNUSED = 1;
    //static public final int UNUSED = 2;
    //static public final int UNUSED = 3;
    //static public final int UNUSED = 4;
    //static public final int UNUSED = 5;
    //static public final int UNUSED = 6;
    //static public final int UNUSED = 7;
    //static public final int UNUSED = 8;
    //static public final int UNUSED = 9;

    // DIO Bank
    static public final int FL_AZMTH_ENC_IDX = 0;
    static public final int FR_AZMTH_ENC_IDX = 1;
    static public final int BL_AZMTH_ENC_IDX = 2;
    static public final int BR_AZMTH_ENC_IDX = 3;
    static public final int ARM_VERTICAL_IDX = 4;
    //static public final int UNUSED = 5;
    //static public final int UNUSED = 6;
    //static public final int UNUSED = 7;
    //static public final int UNUSED = 8;
    //static public final int UNUSED = 9;


    // Analog Bank
    //static public final int UNUSED = 0;
    //static public final int UNUSED = 1;
    //static public final int UNUSED = 2;
    //static public final int UNUSED = 3;

    // CAN Bus Addresses - Motors
    //static public final int RESERVED_DO_NOT_USE = 0; // default for most stuff
    //static public final int RESERVED_DO_NOT_USE = 1; // Rev Power Distribution Hub
    static public final int FL_WHEEL_MOTOR_CANID = 2;
    static public final int FL_AZMTH_MOTOR_CANID = 3;
    static public final int FR_WHEEL_MOTOR_CANID = 4;
    static public final int FR_AZMTH_MOTOR_CANID = 5;
    static public final int BL_WHEEL_MOTOR_CANID = 6;
    static public final int BL_AZMTH_MOTOR_CANID = 7;
    static public final int BR_WHEEL_MOTOR_CANID = 8;
    static public final int BR_AZMTH_MOTOR_CANID = 9;
    static public final int L_FINGER_MOTOR_CANID = 10;
    static public final int R_FINGER_MOTOR_CANID = 11;
    static public final int B_ARM_MOTOR_CANID = 12;

    //static public final int UNUSED = 13;
    //static public final int UNUSED = 12;
    //static public final int UNUSED = 13;
    //static public final int UNUSED = 14;
    //static public final int UNUSED = 15;
    //static public final int UNUSED = 16;
    //static public final int UNUSED = 17;

    // Pneumatics Hub
    //static public final int UNUSED = 0;
    //static public final int UNUSED = 1;
    //static public final int UNUSED = 2;
    //static public final int UNUSED = 3;
    //static public final int UNUSED = 4;
    //static public final int UNUSED = 5;
    //static public final int UNUSED = 6;
    //static public final int UNUSED = 7;
    //static public final int UNUSED = 8;
    //static public final int UNUSED = 9; 

    // PDP Channels - for current measurement
    //static public final int UNUSED = 0;
    //static public final int UNUSED = 1;
    //static public final int UNUSED = 2;
    //static public final int UNUSED = 3;
    //static public final int UNUSED = 4;
    //static public final int UNUSED = 5;
    //static public final int UNUSED = 6;
    //static public final int UNUSED = 7;
    //static public final int UNUSED = 8;
    //static public final int UNUSED = 9;
    //static public final int UNUSED = 10;
    //static public final int UNUSED = 11;
    //static public final int UNUSED = 12;
    //static public final int UNUSED = 13;
    //static public final int UNUSED = 14;
    //static public final int UNUSED = 15;
    //static public final int UNUSED = 16;
    //static public final int UNUSED = 17;
    //static public final int UNUSED = 18;
    //static public final int UNUSED = 19;

    //CAMERA IP: 10.91.6.11

    static public final int CURRENT_LIMIT_WHLMOTOR_AMPS = 40;
    static public final int CURRENT_LIMIT_AZMOTOR_AMPS = 20;

    //////////////////////////////////////////////////////////////////
    // Time-based autonomous Constants
    //////////////////////////////////////////////////////////////////
    public static final double TAXI_DRIVE_TIME_S = 2.2; //YN: might not be even used
    public static final double TAXI_DRIVE_SPEED_MPS = 1.75;

    //////////////////////////////////////////////////////////////////W
    // Nominal Sample Times
    //////////////////////////////////////////////////////////////////
    public static final double Ts = 0.02;
    public static final double SIM_SAMPLE_RATE_SEC = 0.001;

    //////////////////////////////////////////////////////////////////
    // Field Dimensions
    //////////////////////////////////////////////////////////////////
    // todo - Game manual says 26' 3.5" X 54'3/25"
    static public final double FIELD_WIDTH_M = Units.feetToMeters(27.0); //YN: change
    static public final double FIELD_LENGTH_M = Units.feetToMeters(54.0); //YN: change
    static public final Translation2d MAX_ROBOT_TRANSLATION = new Translation2d(FIELD_LENGTH_M, FIELD_WIDTH_M);
    static public final Translation2d MIN_ROBOT_TRANSLATION = new Translation2d(0.0, 0.0);
    // Assumed starting location of the robot. Auto routines will pick their own location and update this.
    public static final Pose2d DFLT_START_POSE = new Pose2d(3, 3, new Rotation2d(0)); //YN: change
    // Expected vision target locations on the field
    // TODO - Use Actual Poses
    static public final Transform3d VISION_FAR_TGT_LOCATION = new Transform3d(new Translation3d(FIELD_LENGTH_M, Units.feetToMeters(9.8541), 1.0), new Rotation3d(0, 0, 0));
    static public final Transform3d VISION_NEAR_TGT_LOCATION = new Transform3d(new Translation3d(Units.feetToMeters(0), Units.feetToMeters(17.14), 1.0), new Rotation3d(0, 0, Math.PI));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////  User Experience
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //CURRENT LIMITS
    public static int OPEN_PHALANGES_CURRENT_A = 2;
    public static int CLOSE_FOR_CUBE_CURRENT_A = 4; //2
    public static int CLOSE_FOR_CONE_CURRENT_A = 15; //2 -> og 8
    public static int STOP_PHALANGES_CURRENT_A = 2;

    //ARM WORK
    public static double FWD_ARM_LIMIT_DEG = -1.0;
    public static double REV_ARM_LIMIT_DEG = -117.0; //og -130.0
    public static double MAGIC_ARM_RATIO = 0.9 * 117.8666666; // We don't know why we need the 0.9 correction factor.
    public static double ARM_PIVOT_CMD_SCALAR_SPEED_RAD_PER_SEC = Constants.MAGIC_ARM_RATIO * (Math.PI * 2.0) * 1.15 * 1.5; // 117.866 gear redution * full unit circle in radians (2pi) * 1.5x speed

    static public final double get_swerve_fwd_rev_cmd_sign() {
        if (Robot.isReal()) {
            return 1.0;
        } else {
            return -1.0;
        }
    }

    static public final double get_swerve_rot_cmd_sign() {
        if (Robot.isReal()) {
            return -1.0;
        } else {
            return -1.0;
        }
    }

    static public final double get_swerve_side_to_side_cmd_sign() {
        if (Robot.isReal()) {
            return 1.0;
        } else {
            return -1.0;
        }
    }

    static public final double SWERVE_FWD_REV_CMD_SIGN = get_swerve_fwd_rev_cmd_sign();
    static public final double SWERVE_ROT_CMD_SIGN = get_swerve_rot_cmd_sign();
    static public final double SWERVE_SIDE_TO_SIDE_CMD_SIGN = get_swerve_side_to_side_cmd_sign();

    static public double ARM_MATCHING_CURRENT_A = 40.0; //YN:TODO546

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////  Derived Constants
    //// - You can reference how these are calculated, but shouldn't
    ////   have to change them
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // HELPER ORGANIZATION CONSTANTS
    static public final int FL = 0; // Front Left Module Index
    static public final int FR = 1; // Front Right Module Index
    static public final int BL = 2; // Back Left Module Index
    static public final int BR = 3; // Back Right Module Index
    static public final int NUM_MODULES = 4;

    static public final int L_FINGER = 0; // L claw module index
    static public final int R_FINGER = 1; // R claw module index
    static public final int NUM_FINGERS = 2;

    static public final double invert_swerve_positions() {
        double result;
        if (Robot.isReal()) {
            result = -1.0;
        } else {
            result = 1.0;
        }
        return result;
    }
    static public final double I_POS = invert_swerve_positions();

    // Internal objects used to track where the modules are at relative to
    // the center of the robot, and all the implications that spacing has.
    static public final List < Translation2d > robotToModuleTL = Arrays.asList(
        new Translation2d(+Constants.HALF_LENGTH_M * I_POS, +Constants.HALF_WIDTH_M * I_POS),
        new Translation2d(+Constants.HALF_LENGTH_M * I_POS, -Constants.HALF_WIDTH_M * I_POS),
        new Translation2d(-Constants.HALF_LENGTH_M * I_POS, +Constants.HALF_WIDTH_M * I_POS),
        new Translation2d(-Constants.HALF_LENGTH_M * I_POS, -Constants.HALF_WIDTH_M * I_POS)
    );

    static public final List < Transform2d > robotToModuleTF = Arrays.asList(
        new Transform2d(robotToModuleTL.get(FL), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(FR), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(BL), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(BR), new Rotation2d(0.0))
    );

    static public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        robotToModuleTL.get(FL),
        robotToModuleTL.get(FR),
        robotToModuleTL.get(BL),
        robotToModuleTL.get(BR)
    );

}