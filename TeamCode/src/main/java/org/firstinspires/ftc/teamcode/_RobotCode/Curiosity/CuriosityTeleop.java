package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;


@TeleOp(name = "*CURIOSITY TELEOP*", group = "Curiosity")
@Config
public class CuriosityTeleop extends OpMode implements ControllerInputListener
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
    public static double odometryTestY = 12;

    public static double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;



    @Override
    public void init() {
        robot = new CuriosityBot(this,true,false,false);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        //hardwareMap.dcMotor.get("FR").setDirection(DcMotorSimple.Direction.REVERSE);
        //hardwareMap.dcMotor.get("FL").setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        msStuckDetectLoop = 5000;
    }

    @Override
    public void start(){
        robot.start();
        robot.getChassis().resetGyro();
        //if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE) robot.SetInputOffset(90); //90 is blue, -90 is red
        //else if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.RED) robot.SetInputOffset(-90); //90 is blue, -90 is red
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
        telemetry.addData("Intake: ", "Hold LT");
        telemetry.addData("Load: ", "Hold RT");
        telemetry.addData("Toggle shooter: ", "Press Y");
        telemetry.addData("Toggle intake: ", "Press RB");
        telemetry.addData("Toggle path: ", "Press LB");


        /*//DATA
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
        telemetry.addLine();*/
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
            case B:// reset robot pose
                break;
            case LJS:// toggle headless
                //robot.getChassis().switchHeadlessMode();
                break;
            case RJS:// reset robot pose
                robot.getNavigator().setRobotPose(0, 0, 0);
                robot.getNavigator().getChassis().driveMotors.StopAndResetEncoders();
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
