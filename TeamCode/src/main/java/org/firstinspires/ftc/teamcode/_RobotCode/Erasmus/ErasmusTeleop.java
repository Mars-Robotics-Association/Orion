package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode._RobotCode.Demobot2022.Demobot;


@TeleOp(name = "*ERASMUS TELEOP*", group = "Demobot")
@Config
public class ErasmusTeleop extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private Erasmus robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = 1;//used to change how fast robot turns
    public static double odometryTestSpeed = -0.5;
    public static double odometryTestAngle = 180;
    public static double odometryTestX = 12;
    public static double odometryTestY = 12;

    private double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;



    @Override
    public void init() {
        robot = new Erasmus(this,true,true,true);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        /*hardwareMap.dcMotor.get("FR").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("FL").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("RR").setDirection(DcMotor.Direction.REVERSE);
        hardwareMap.dcMotor.get("RL").setDirection(DcMotor.Direction.REVERSE);*/

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        msStuckDetectLoop = 5000;
    }

    @Override
    public void start(){
        robot.start();
        robot.getChassis().resetGyro();
        robot.getChassis().setHeadlessMode(false);
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        controllerInput2.Loop();
        //navigator kill switch
        if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {

        }
        //update robot
        robot.update();
        //manage driving
        robot.getChassis().driveWithGamepad(controllerInput1, driveSpeed, turnSpeed, speedMultiplier);
        //telemetry
        printTelemetry();
        telemetry.update();
    }

    //prints a large amount of telemetry for the robot
    private void printTelemetry() {
        //CONTROLS
        telemetry.addLine("----CONTROLS----");
        //chassis
        telemetry.addLine("Chassis");
        telemetry.addData("Drive with: ", "LJS");
        telemetry.addData("Turn with: ", "RJS");
        telemetry.addData("Change speed multiplier: ", "A");
        telemetry.addData("Reset robot pose: ", "Press RJS");
        telemetry.addData("Toggle headless mode: ", "Press LJS");
        //payload
        telemetry.addLine("Payload");
        telemetry.addData("Move arm: ", "Triggers");
        telemetry.addData("Toggle gripper: ", "Press RB");



        //DATA
        telemetry.addLine();
        telemetry.addLine("----DATA----");
        //Dead wheel positions
        telemetry.addLine("Dead wheel positions");
        double[] deadWheelPositions = robot.getNavigator().getDeadWheelPositions();
        telemetry.addData("LEFT dead wheel: ", deadWheelPositions[0]+" inches");
        telemetry.addData("RIGHT dead wheel: ", deadWheelPositions[1]+" inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2]+" inches");
        //Odometry estimated pose
        telemetry.addLine();
        telemetry.addLine("Robot pose");
        Pose2d robotPose = robot.getNavigator().getPose();
        telemetry.addData("X, Y, Angle", robotPose.getX() + ", " + robotPose.getY() + ", " + Math.toDegrees(robotPose.getHeading()));
        telemetry.addLine();
    }

    @Override
    public void stop(){
        robot.stop();
    }

    ////INPUT MAPPING////
    @Override
    public void ButtonPressed(int id, Button button) {
        switch (button) {
            case A:// speed multiplier cycling
                if (speedMultiplier == 1) speedMultiplier = 0.5;
                else speedMultiplier = 1;
                break;
            case B:
                break;
            case LJS:// toggle headless
                robot.getChassis().switchHeadlessMode();
                break;
            case RJS:// reset robot pose
                robot.getNavigator().setRobotPose(0, 0, 0);
                robot.getNavigator().getChassis().driveMotors.StopAndResetEncoders();
                robot.getChassis().resetGyro();
                break;
            case RB:
                if(robot.USE_PAYLOAD) robot.getPayload().toggleGripper();

        }
    }

    @Override
    public void ButtonHeld(int id, Button button) {
        switch (button){
            case RT:
                //robot.getNavigator().goTowardsPose(odometryTestX,odometryTestY,odometryTestAngle,odometryTestSpeed);
                if(robot.USE_PAYLOAD)robot.getPayload().moveArm(0.25);
                break;
            case LT:
                //robot.getNavigator().turnTowards(odometryTestAngle,odometryTestSpeed);
                if(robot.USE_PAYLOAD)robot.getPayload().moveArm(-0.25);
                break;

        }
    }

    @Override
    public void ButtonReleased(int id, Button button) {
        switch (button){
            case RT:
            case LT:
                if(robot.USE_PAYLOAD) robot.getPayload().moveArm(0);
                break;
        }
    }


}
