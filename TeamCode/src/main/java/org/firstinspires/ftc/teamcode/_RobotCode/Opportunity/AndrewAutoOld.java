package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@Disabled
@Config
@Autonomous(name = "Andrew auto but ~newer~ ", group = "All")
public class AndrewAutoOld extends LinearOpMode
{
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RR;
    private DcMotor RL;
    private DcMotor armPos;
    private DcMotor gripper;
    private DcMotor turntable;
    private final Object       runningNotifier = new Object();


    private IMU imu;
    private AndrewIMU andrewIMU;
    private DcMotor duckyMotor;
    private ColorSensor colorBottom;
    private ColorSensor colorSide1;
    private DistanceSensor sideDist;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMU(this);
      initialStuff();
      waitForStart();
        imu.ResetGyro();

        setWheelSpeedsTurning(270,0.5,0); //initial forward
        waitForMotors(new DcMotor[] {FR,FL,RR,RL},850);
        sleep(200);

        setWheelSpeedsTurning(180,0.4,0); // slide right to red line
        ///waitForColor(colorBottom,75,500,500);
        waitForMotors(new DcMotor[] {FR,FL,RR,RL},300);

        sleep(200);

        setWheelSpeedsTurning(270,0.4,0); // nudge forward
        waitForMotors(new DcMotor[] {FR,FL,RR,RL},100);
        sleep(200);

        duckyMotor.setPower(-0.7);
        setWheelSpeedsTurning(0,0.4,0); //move into turntable
        waitForMotors(new DcMotor[] {FR,FL,RR,RL},370);
        sleep(4500);
duckyMotor.setPower(0);

        setWheelSpeedsTurning(180,0.4,0); // move back, slightly past red line
      //  waitForColor(colorBottom,75,500,500);
        waitForMotors(new DcMotor[] {FR,FL,RR,RL},500);
        stopWheels();
        sleep(300);

        setWheelSpeedsTurning(0,0,0.3);
    //    waitForIMU(imu, 0, true);
        stopWheels();

/*
        setWheelSpeedsTurning(75,0.7,0); // parking
        waitForMotors(new DcMotor[] {FR,FL,RR,RL},2500);
        stopWheels();

        setWheelSpeedsTurning(45,0.5,-0.05);  // 45 degree move back
        waitForMotors(new DcMotor[] {FR,FL,RR,RL},1500);
        stopWheels();
        sleep(200);

        setWheelSpeedsTurning(75,0.5,0); // parking
        waitForMotors(new DcMotor[] {FR,FL,RR,RL},4000);
        stopWheels();
        sleep(200);
*/
    }

    /**
     * Initialize hardware
     *
     */
    void initialStuff(){
        FR = this.hardwareMap.dcMotor.get("FR");
        FL = this.hardwareMap.dcMotor.get("FL");
        RR = this.hardwareMap.dcMotor.get("RR");
        RL = this.hardwareMap.dcMotor.get("RL");
        duckyMotor = this.hardwareMap.dcMotor.get("duckyMotor");
        armPos = this.hardwareMap.dcMotor.get("armPosition");
        turntable = this.hardwareMap.dcMotor.get("turntable");
        sideDist = hardwareMap.get(DistanceSensor.class, "distSide");




    //    colorBottom = hardwareMap.colorSensor.get("color1"); // bottom
    //    colorSide1 = hardwareMap.colorSensor.get("color2"); //side

    }

    /**
     *
     * Waits for motors to travel a specified distance
     * @param motors
     * @param targetPosition
     */
    void waitForMotors(DcMotor[] motors, int targetPosition){
        int avgDistance = 0;
        int sum = 0;
        int[] initialPositions = new int[motors.length];
        for(int i = 0; i<motors.length; i++)
            initialPositions[i] = motors[i].getCurrentPosition();
        while (avgDistance<targetPosition) {
            if(!opModeIsActive()) return;
            telemetry.addData("avgDistance",avgDistance);
            telemetry.addData("sum",sum);
            telemetry.addData("targetPosition",targetPosition);
            telemetry.addData("initial[0]",initialPositions[0]);
            telemetry.addData("motor0",motors[0].getCurrentPosition());
            telemetry.update();

//            synchronized (runningNotifier) {
//                try {
//                    runningNotifier.wait();
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                    return;
//                }
//            }
            sum = 0;
            for(int i = 0; i<motors.length; i++)
                sum+=Math.abs(motors[i].getCurrentPosition()-initialPositions[i]);
            avgDistance = sum/motors.length;
        }
        return;
    }

    /**
     * Waits for color sensor values to pass a certain threshold
     * Ex: to wait until red is found, use something like waitForColor(color, 80, 500, 500);
     * @param color
     * @param red
     * @param green
     * @param blue
     */
    void waitForColor(ColorSensor color,int red, int green, int blue){
        while (!(color.red()>red || color.green()>green || color.blue()>blue)) {
            if (!opModeIsActive()) return;
            telemetry.addData("red",color.red());
            telemetry.addData("green",color.red());
            telemetry.addData("blue",color.red());
            telemetry.update();
        }

    }

    void waitForIMU(IMU passedIMU, double angle, boolean moreThan){
        while (passedIMU.GetRobotAngle() < angle == moreThan) {
            if (!opModeIsActive()) return;
            telemetry.addData("angle",passedIMU.GetRobotAngle());
            telemetry.update();
        }

    }



    void waitForDistanceSensor(DistanceSensor dist, double max, double min){
        while (dist.getDistance(DistanceUnit.INCH)<max && dist.getDistance(DistanceUnit.INCH)>min) {
            if (!opModeIsActive()) return;
            telemetry.addData("dist",dist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }

    /**
     * Sets robot speed and turning speed
     * @param degrees
     * @param speed
     * @param turnSpeed
     */
    void setWheelSpeedsTurning(double degrees, double speed, double turnSpeed){
        double[] newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(degrees,speed,turnSpeed);
        FR.setPower(newSpeeds[0]);
        FL.setPower(newSpeeds[2]);
        RR.setPower(newSpeeds[1]);
        RL.setPower(newSpeeds[3]);
    }

    void stopWheels(){
        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);
    }





}
