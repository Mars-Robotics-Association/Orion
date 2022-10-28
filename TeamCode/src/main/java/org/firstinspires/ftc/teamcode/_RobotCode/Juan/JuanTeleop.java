package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Navigation.Camera;


@TeleOp(name = "*JUAN TELEOP*", group = "JUAN")
@Config
public class JuanTeleop extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private Juan robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;
    private Camera camera;
    private FtcDashboard dash;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = 1;//used to change how fast robot turns

    private double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;

    @Override
    public void init() {
        robot = new Juan(this,true,true,false);
        camera = new Camera(this,"Webcam 1",false);
        dash = FtcDashboard.getInstance();
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

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

        try {
            dash.sendImage(camera.GetImage());
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

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
        telemetry.addData("Drive with: ", "LJS");
        telemetry.addData("Turn with: ", "RJS");
        telemetry.addData("Change speed multiplier: ", "A");
        telemetry.addData("Reset robot pose: ", "Press RJS");
        telemetry.addData("Toggle headless mode: ", "Press LJS");

        if(robot.getChassis().isUSE_PAYLOAD())robot.getPayload().printTelemetry();

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
            case LJS:
                robot.getChassis().switchHeadlessMode();
                break;
            case RJS:// reset robot pose
                robot.getChassis().driveMotors.StopAndResetEncoders();
                robot.getChassis().resetGyro();
                break;
        }
        if(robot.USE_PAYLOAD) {
            JuanPayload payload = robot.getPayload();
            JuanPayload.LiftController lift = payload.getLift();
            JuanPayload.GripperController gripper = payload.getGripper();

            switch (button) {
                case A:
                    lift.goToAbsoluteBottom();
                    break;
                case X:
                    lift.goToPreset(JuanPayload.LiftController.LiftHeight.LOW);
                    break;
                case Y:
                    lift.goToPreset(JuanPayload.LiftController.LiftHeight.MEDIUM);
                    break;
                case B:
                    lift.goToPreset(JuanPayload.LiftController.LiftHeight.HIGH);
                    break;
                case LT:
                    gripper.grab();
                    break;
                case RT:
                    gripper.release();
                    break;
            }
        }
    }

    @Override
    public void ButtonHeld(int id, Button button) {
        switch (button){

        }
    }

    @Override
    public void ButtonReleased(int id, Button button) {
        switch (button){

        }
    }


}
