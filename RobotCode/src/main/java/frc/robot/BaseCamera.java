package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BaseCamera {
    NetworkTable limelight;
    public NetworkTableEntry tx, ty, ta, ts, tv, pipeline, camMode;

    public BaseCamera(String name) {
        NetworkTableInstance defaultNetworkTable = NetworkTableInstance.getDefault();
        limelight = defaultNetworkTable.getTable("limelight" + name);
        update();
    }

    public void update() {
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        ts = limelight.getEntry("ts");
        tv = limelight.getEntry("tv");
        pipeline = limelight.getEntry("pipeline");
        camMode = limelight.getEntry("camMode");
    }

    public void setPipeline(int x) {
        pipeline.setNumber(x);
    }

    public void setCamMode(int x) {
        camMode.setNumber(x);
    }

    public double getX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return ty.getDouble(0.0);
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    public double getSkew() {
        return ts.getDouble(0.0);
    }

    public long seesTarget() {
        long testingLong = tv.getInteger(0);
        return testingLong;
    }
}