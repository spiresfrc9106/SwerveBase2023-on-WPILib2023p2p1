# Spires FRC team number 9106 source code for the 2022-2023, Energize, Charged Up Game

This source code was forked from https://github.com/gerth2/SwerveBase2023 which is based upon https://github.com/RobotCasserole1736/RobotCasserole2022 .

The Spires would like to thank RobotCasserole - FRC 1736 for publishing this execllent swerve source code and giving us advice on how to use the code base!

[Video of our autonoumous place, mobilitiy, and engage, then in teleop, jumping the charging station in our final East Kentwood District Match](https://www.youtube.com/watch?v=HQKJHTP1DtY).

# Some Caution

As a rookie team, when we modified the code base we didn't keep it up to the same high quality of the initial RobotCaserole code base. View our changes with caution.

The code base has a localhost webserver that runs on the RoboRIO website that can used for sims and robot testing.  To our knowledge the webserver does not work well with Network Tables 4. (Just a heads up you'll need running code in simulation for the link to work.) [Click here if you dare](http://localhost:5805/).

It is our feeling that in 

[RealSparkMax.java](./RobotCode/src/main/java/frc/hardwareWrappers/MotorCtrl/SparkMax/RealSparkMax.java)

and in

[RealTalonFX.java](./RobotCode/src/main/java/frc/hardwareWrappers/MotorCtrl/TalonFX/RealTalonFX.java)

there is 

```
        // I don't know why we need to do this, but it makes sim line up with real life.
        p /= 1000;

        //Convert to Rev units of RPM
        p = Units.radiansPerSecondToRotationsPerMinute(p);
        i = Units.radiansPerSecondToRotationsPerMinute(i);
        d = Units.radiansPerSecondToRotationsPerMinute(d);

        m_pidController.setP(p);
        m_pidController.setI(i);
        m_pidController.setD(d);
        m_pidController.setOutputRange(-1.0, 1.0);
```

and there is:

```
        // I don't know why we need to do this, but it makes sim line up with real life.
        p /= 1000;

        //Convert to CTRE Units
        p = ( CMD_PER_V ) *  RevtoCTRENativeUnits(Units.radiansToRotations(p));
        i = ( CMD_PER_V ) *  RevtoCTRENativeUnits(Units.radiansToRotations(i)); //CTRE needs this in * 1000 ms (or * 1 sec)
        d = ( CMD_PER_V ) *  RevtoCTRENativeUnits(Units.radiansToRotations(d)); //CTRE needs this in / 1000 ms (or / 1 sec)

        _talon.config_kP(0, p, TIMEOUT_MS);
        _talon.config_kI(0, i, TIMEOUT_MS);
        _talon.config_kD(0, d, TIMEOUT_MS);
```

that for the SPARK MAX this makes the integral and derivative gain 1000 times stronger than the proporational gain, making it very confusing when setting the gains from higher level routines.  We suspect this is true for the Talon's as well.

This should be resovled. Use caution when adjusting the integral gains with powerful motors.

Also because our robot did not have any Talon motor controllers, and we were pressed for time, when we added features for the SPARK MAX controllers, we did not make the equivalent functionality work in the Talon controllers.

We had some problems with the code that monitored the power distribution hub, we disabled it, and never re-enabled it.

We got the robot driving really well both field relative and robot relative with the RobotCaserole code base, we think we configured our robot to be upside down relative to the standard FRC field coordinate system. But because we never used cameras to update our odometery position, this did not matter for our use case.


# Adjustments to the RobotCasserole Code Base
We adjusted the RobotCasserole code base to use (some support for this configuration was in the code, we mostly configured the code for our configuration):

* REV Robotics 3 inch MaxSwerve  https://www.revrobotics.com/rev-21-3005/ with:
  * Rev Robotics Neo Brushless DC Motors https://www.revrobotics.com/rev-21-1650/
  * Rev Robotics Neo 550 Brushless DC Motors https://www.revrobotics.com/rev-21-1651/
  * Rev Robotis Through Bore Encoders https://www.revrobotics.com/rev-11-1271/
  * Rev Robotics SPARK MAX Motor Controllers: https://www.revrobotics.com/rev-11-2158/

* We used a navX2-MXP Robotics Navigation Sensor placed in the RoboRIO Expansion IO: https://www.kauailabs.com/store/index.php?route=product/product&product_id=69

We also added a pivot arm with:

  * Another Rev Robotics Neo Brushless DC Motors https://www.revrobotics.com/rev-21-1650/
  * About 117:1 gear reduction
  * Another Rev Robotis Through Bore Encoders https://www.revrobotics.com/rev-11-1271/

We also added a claw with two fingers:
  * Each finger has:
    * A Rev Robotics Neo 550 Brushless DC Motors https://www.revrobotics.com/rev-21-1651/
    * About 12:1 gear reduction.

We also added a MB1043 sonar sensor which is equivalent to the MB1122 https://maxbotix.com/pages/hrlv-maxsonar-ez-datasheet

We also added a Limelight 2+ https://limelightvision.io/products/limelight-2-plus with a google coral https://coral.ai/ 

# Configuration of the RobotCasserole Code Base

Most configuration of the code base is in [Constants.java](./RobotCode/src/main/java/frc/Constants.java) .

Indexes of the 4 swerve modules in arrays in the code:

```
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // HELPER ORGANIZATION CONSTANTS
    static public final int FL = 0; // Front Left Module Index
    static public final int FR = 1; // Front Right Module Index
    static public final int BL = 2; // Back Left Module Index
    static public final int BR = 3; // Back Right Module Index
    static public final int NUM_MODULES = 4;
```


Our robot was not square so we have both a `HALF_WIDTH_M` and a `HALF_LENGTH_M` in Physical Drivetrain constants:

```
    //////////////////////////////////////////////////////////////////
    // Drivetrain Physical
    //////////////////////////////////////////////////////////////////
    static public final double HALF_WIDTH_M = Units.inchesToMeters(16.5/2.0);
    static public final double HALF_LENGTH_M = Units.inchesToMeters(26.5/2.0);
    static public final double WHEEL_BASE_HALF_WIDTH_M = Units.inchesToMeters(26.5/2.0); // todo - work towards removing this.
    static public final double ROBOT_MASS_kg = UnitUtils.lbsToKg(100); //YN: change 100 to ---?
    static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_HALF_WIDTH_M*2.2),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center

    // Drivetrain Performance Mechanical limits
    static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(19.0); //YAVIN-NOTE: 19/13x faster
    static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(19.0); //YAVIN-NOTE: 19/13x faster
    static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(180.0); //YAVIN-NOTE: 2x faster
    static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/1.00; //0-full time of 0.25 second
    static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/.25; //0-full time of 0.25 second

    // See https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081
    static public final double WHEEL_GEAR_RATIO = 4.71; //L2 gearing
    static public final double AZMTH_GEAR_RATIO = 4.71; //todo - fix me up
    static public final double WHEEL_FUDGE_FACTOR = 0.9238; // carpet roughtop scrub factor
    static public final double WHEEL_RADIUS_IN = 3.0/2.0 * WHEEL_FUDGE_FACTOR; //THREE inch diameter wheels - https://www.swervedrivespecialties.com/collections/mk4i-parts/products/billet-wheel-4d-x-1-5w-bearing-bore
```


Configuration of the PWM outputs of the Through Bore Encoders to the RoboRIO DIO PWM inputs:


```
    // DIO Bank
    static public final int FL_AZMTH_ENC_IDX = 0; 
    static public final int FR_AZMTH_ENC_IDX = 1;
    static public final int BL_AZMTH_ENC_IDX = 2;
    static public final int BR_AZMTH_ENC_IDX = 3;
    static public final int ARM_VERTICAL_IDX = 4;
```

CAN Bus Addresses:

```
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
```

# Contact Information:

Our team can be reached at: ccrobotics@grcatholiccentral.org
