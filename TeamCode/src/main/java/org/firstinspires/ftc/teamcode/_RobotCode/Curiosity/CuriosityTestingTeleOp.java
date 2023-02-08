package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;


@TeleOp(name = "Curiosity Testing Teleop", group = "Curiosity")
@Config
public class CuriosityTestingTeleOp extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private CuriosityBot robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double odometryTestSpeed = 0.8;
    public static double odometryTestAngle = 180;
    public static double odometryTestX = 12;
    public static double odometryTestY = 0;

    public static double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;

    //Reference
    private double lastRuntime = 0;
    double armInput = 0;
    boolean isBusy = false; //use to override usual user drive input



    @Override
    public void init() {
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);
        robot = new CuriosityBot(this,controllerInput1,true,false,true,true);
        robot.getChassis().setInputOffset(0);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        msStuckDetectLoop = 5000;
    }

    @Override
    public void start(){
        robot.start();
        robot.getNavigator().setMeasuredPose(0, 0, 0);
        robot.getNavigator().getChassis().driveMotors.stopAndResetEncoders();
        robot.getChassis().resetGyro();
        robot.getChassis().setHeadlessMode(true);
    }

    @Override
    public void loop() {
        controllerInput1.loop();
        controllerInput2.loop();
        //navigator kill switch
        if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {
            robot.stop();
        }
        //update robot
        try {
            robot.update();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //manage driving
        if(!isBusy) {
            robot.getChassis().driveWithGamepad(controllerInput1, speedMultiplier);
        }

        //telemetry
        printTelemetry();
        telemetry.update();
        lastRuntime = getRuntime();

    }

    //prints a large amount of telemetry for the robot
    private void printTelemetry() {
        //CONTROLS
        telemetry.addLine("----CONTROLS----");
        telemetry.addData("Drive with: ", "LJS");
        telemetry.addData("Turn with: ", "RJS");
        telemetry.addData("Change speed multiplier: ", "A");
        telemetry.addData("Reset robot pose: ", "Press RJS");
        telemetry.addData("Toggle headless mode: ", "Press LJS");
        telemetry.addData("Gripper: ", "Right bumper");
        telemetry.addData("Arm: ", "Triggers");
        telemetry.addData("Load: ", "Press B");
        telemetry.addData("Place: ", "Press Y");
        telemetry.addData("Arm manual mode: ", "Press X");
        telemetry.addData("Change heights: ", "Dpad");
        telemetry.addData("Nudge single arm motor: ", "Left bumper, hold x to nudge down nothing to nudge up");


        //DATA
        telemetry.addLine();
        telemetry.addLine("----DATA----");
        //runtime
        telemetry.addData("Loop time ms: ", (getRuntime()-lastRuntime)*1000);
        //Dead wheel positions
        telemetry.addLine("Dead wheel positions");
        double[] deadWheelPositions = robot.getNavigator().getDeadWheelPositions();
        telemetry.addData("LEFT dead wheel: ", deadWheelPositions[0]+" inches");
        telemetry.addData("RIGHT dead wheel: ", deadWheelPositions[1]+" inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2]+" inches");
        //Odometry estimated pose
        telemetry.addLine();
        telemetry.addLine("Robot pose");
        Pose2d robotPose = robot.getNavigator().getMeasuredPose();
        telemetry.addData("X, Y, Angle", robotPose.getX() + ", " + robotPose.getY() + ", " + Math.toDegrees(robotPose.getHeading()));
        //arm
        telemetry.addLine();
        telemetry.addData("Arm Input", armInput);
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
            case LJS:// toggle headless
                robot.getChassis().switchHeadlessMode();
                break;
            case RJS:// reset robot pose
                robot.getNavigator().setMeasuredPose(0, 0, 0);
                robot.getNavigator().getChassis().driveMotors.stopAndResetEncoders();
                robot.getChassis().resetGyro();
                break;

            //PAYLOAD TESTING
            case Y:
                robot.getPayload().toggleGripper();
                break;
        }
    }

    @Override
    public void ButtonHeld(int id, Button button) {
        switch (button){
            case Y:
                isBusy = true;
                robot.getNavigator().goTowardsPose(odometryTestX,odometryTestY,odometryTestAngle,odometryTestSpeed);
                break;
            case X:
                isBusy = true;
                robot.getNavigator().moveTowards(odometryTestX,odometryTestY,odometryTestSpeed);
                break;
            case B:
                isBusy = true;
                robot.getNavigator().turnTowards(odometryTestAngle,odometryTestSpeed);
                break;

            //PAYLOAD TESTING
            case RT:
                robot.getPayload().getArm().setPowerRaw(1);
                break;
            case LT:
                robot.getPayload().getArm().setPowerRaw(-1);
                break;
            case RB:
                robot.getPayload().getLift().setPowerRaw(1);
                break;
            case LB:
                robot.getPayload().getLift().setPowerRaw(-1);
                break;
        }
    }

    @Override
    public void ButtonReleased(int id, Button button) {
        switch (button){
            case Y:
            case B:
            case X:
                isBusy = false;
                break;

            //PAYLOAD TESTING
            case RT:
            case LT:
                robot.getPayload().getArm().setPowerRaw(0);
                break;
            case RB:
            case LB:
                robot.getPayload().getLift().setPowerRaw(0);
                break;

        }
    }


}
