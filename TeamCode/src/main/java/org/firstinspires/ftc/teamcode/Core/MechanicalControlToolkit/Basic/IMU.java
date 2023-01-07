package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMU
{
    // Dependenices
    private BNO055IMU imu;
    private OpMode opMode;

    // Variables
    private Orientation angles;
    private Acceleration gravity;
    private double offset;

    public IMU (OpMode setOpMode) {
        opMode = setOpMode;
    }

    public void start() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //parameters.accelerationIntegrationAlgorithm = new BasicAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);
    }


    ////GETTERS////
    public Orientation getRawAngles()
    {
        //return the raw angles from the imu
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles;
    }
    public double getRobotAngle(){
        //return the firstangle from the imu with an offset applied from ResetGyro()
        return getRawAngles().firstAngle - offset;
    }
    public double getAngularVelocity(){
        return imu.getAngularVelocity().xRotationRate;
    }
    public Velocity getVelocity(){return imu.getVelocity();}

    public Acceleration getAcceleratioin(){
        return imu.getAcceleration();
    }
    public double calculateDriftAngle(){
        /*double Y = GetAcceleratioin().yAccel;
        double X = GetAcceleratioin().xAccel;

        double heading = Math.atan2(Y,X); //get measurement of joystick angle
        heading = Math.toDegrees(heading);
        heading -= 270;
        if(heading < 0)//convert degrees to positive if needed
        {
            heading = 360 + heading;
        }
        return heading;*/
        return 0;
    }
    public Acceleration getGravity()
    {
        gravity  = imu.getGravity();
        return gravity;
    }

    ////CALLABLE METHODS////
    public void resetGyro(){
        offset = getRawAngles().firstAngle;
    }
    public void offsetGyro(double offsetDegrees){offset+=offsetDegrees;}
}
