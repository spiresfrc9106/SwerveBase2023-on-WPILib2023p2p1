package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BaseCamera {

    /**
    * Create an instance of a base camera.
    *
    * @return Newly created Limelight instance
    */

    NetworkTable limelight;
    public NetworkTableEntry tx,ty,ta,ts,tv,pipeline,camMode;

    public BaseCamera(String name) {
        NetworkTableInstance defaultNetworkTable = NetworkTableInstance.getDefault();

        limelight = defaultNetworkTable.getTable("limelight"+name);

        //NetworkTableInstance newCam = NetworkTableInstance.create();
        //NetworkTable limelight = newCam.getTable("limelight");
        update();

        //reference https://docs.limelightvision.io/en/latest/networktables_api.html for details
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
        //System.out.println("seeing target method ran");
        long testingLong = tv.getInteger(0);
        return testingLong;
    }
    
    /* public void distanceEst() {
          USE THIS FOR ESTIMATING DISTANCE

            double targetOffsetAngle_Vertical = ty.getDouble(0.0);

            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = 25.0;
            
            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = 20.0;
            
            // distance from the target to the floor
            double goalHeightInches = 60.0;
            
            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
            
            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    } */
}
