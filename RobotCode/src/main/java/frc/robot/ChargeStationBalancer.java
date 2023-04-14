package frc.robot;
import frc.robot.AutoDrive.AutoDrive;
import frc.hardwareWrappers.Gyro.WrapperedGyro;
import frc.lib.Signal.Annotations.Signal;

public class ChargeStationBalancer {
    public static final double PITCH_THRESHOLD = 2.0; //deg
    public static final double PITCH_KP = (0.02 / 3) * 0.51; //pitch proportional gain
    public WrapperedGyro gyro;
    AutoDrive ad;
    @Signal
    double pitchError_deg = 0;
    @Signal
    double pitchSpeed = 0;
    @Signal
    double prevPitch = 0;
    @Signal
    double curPitch = 0;

    public ChargeStationBalancer(WrapperedGyro givenGyro) {
        gyro = givenGyro;
        ad = AutoDrive.getInstance();
    }

    public boolean balanceOnChargeStation() {
        System.out.println("pitch speed: " + pitchSpeed);
        boolean balanced = isBalanced();
        if (!balanced) {
            pitchError_deg = getPitchAngle_deg() - 0.0;
            pitchSpeed = pitchError_deg * PITCH_KP;
            double forwd = 0;
            double right = 0;
            if (Math.abs(pitchError_deg) > PITCH_THRESHOLD) {
                forwd = pitchSpeed;
            }
            ad.transformRobot(forwd, right, 0, true);
        } else {
            ad.transformRobot(0, 0, 0, true);
        }
        return balanced;
    }

    public boolean isBalanced() {
        double pitchAngle = getPitchAngle_deg();
        return Math.abs(pitchAngle) < PITCH_THRESHOLD;
    }

    public double getPitchAngle_deg() {
        curPitch = -gyro.getRoll_deg();
        if (Math.abs(curPitch - prevPitch) > 5) {
            return prevPitch;
        } else {
            prevPitch = curPitch;
            return curPitch;
        }
    }

    public double getRollAngle_deg() {
        return gyro.getPitch_deg();
    }
}