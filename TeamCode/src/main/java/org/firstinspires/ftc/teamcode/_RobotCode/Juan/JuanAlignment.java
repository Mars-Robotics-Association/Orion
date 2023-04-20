package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Navigation.UniversalThreeWheelNavigator;

@TeleOp(name = "*JUAN ALIGNMENT*", group = "JUAN")
@Config
public class JuanAlignment extends OpMode
{
    ////Dependencies////
    private Juan robot;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns
    private final double speedMultiplier = 1;

    private final double liftOverrideSpeed = 5;

    public static int payloadControllerNumber = 1;

    public final double       DISTANCE_TARGET = 29.625;
    public final DistanceUnit DISTANCE_UNIT   = DistanceUnit.INCH;

    DistanceSensor frontDistanceSensor;
    DistanceSensor backDistanceSensor;

    @Override
    public void init() {
        telemetry.setAutoClear(false);

        telemetry.addData("Speed Multiplier", speedMultiplier);

        robot = new Juan(this, true, true, true);

        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "distanceFront");
        backDistanceSensor  = hardwareMap.get(DistanceSensor.class, "distanceBack");

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        UniversalThreeWheelNavigator.stopDistance = 0;
    }

    @Override
    public void start(){
        robot.start();
        robot.getChassis().resetGyro();
        robot.getChassis().setHeadlessMode(false);
    }

    @Override
    public void loop() {
        telemetry.addData("Version", Juan.VERSION);

        double distance1 = backDistanceSensor.getDistance(DISTANCE_UNIT);
        double distance2 = frontDistanceSensor.getDistance(DISTANCE_UNIT);
        double distanceCorrection = ((distance1 + distance2) / 2) - DISTANCE_TARGET;
        double angleCorrection = distance1 - distance2;

        telemetry.addData("Distance Correction", distanceCorrection);
        telemetry.addData("Angle Correction", angleCorrection);

        JuanNavigation nav = robot.navigator;

        Pose2d pose = nav.getMeasuredPose();
        nav.goTowardsPose(0, distanceCorrection, 0, 1);

        //update robot
        robot.update();

        //telemetry
        printTelemetry();
        telemetry.update();
    }

    //prints a large amount of telemetry for the robot
    private void printTelemetry() {
        if(robot.USE_PAYLOAD)robot.getPayload().printTelemetry();
    }

    @Override
    public void stop(){
        UniversalThreeWheelNavigator.stopDistance = .2;
        telemetry.addData("line", "STOP");
        telemetry.update();
        robot.stop();
    }
}
