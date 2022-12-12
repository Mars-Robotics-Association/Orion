package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;

//@Disabled
@TeleOp(name = "*INGENUITY Demo*", group = "ingenuity")
@Config
public class IngenuityDemo extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private IngenuityDemoRobot robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;
    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns

    private double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;

    public static double armPower = 0.5 ;

    @Override
    public void init() {
        robot = new IngenuityDemoRobot(this,true,true,true);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        //hardwareMap.dcMotor.get("FR").setDirection(DcMotorSimple.Direction.REVERSE);
        //hardwareMap.dcMotor.get("FL").setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        msStuckDetectLoop = 10000 ;  // Default = 5000
    }

    @Override
    public void start(){
        robot.start();
        robot.getChassis().resetGyro();
        robot.getChassis().setHeadlessMode(true);
    }

    @Override
    public void loop() {
        controllerInput1.loop();
        controllerInput2.loop();
        //navigator kill switch
        if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {

        }
        //update robot
        robot.update();
        //manage driving
        robot.getChassis().driveWithGamepad(controllerInput1, speedMultiplier);
        //telemetry
        printTelemetry();
        //telemetry.update();
    }

    //prints a large amount of telemetry for the robot
    private void printTelemetry() {
        /*
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

        robot.getPayload().printTelemetry();
        */
        //DATA
        telemetry.addLine();
        telemetry.addLine("----DATA----");
        telemetry.addData("Gripper: ", robot.servoTarget);
        telemetry.addData("Arm:     ", robot.armMotor.getCurrentPosition());
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
        telemetry.addLine();
    }

    @Override
    public void stop(){
        robot.stop();
    }

    ////INPUT MAPPING////
    @Override
    public void ButtonPressed(int id, ControllerInput.Button button) {
        switch (button) {
            case A:// speed multiplier cycling
                if (speedMultiplier == 1) speedMultiplier = 0.5;
                else speedMultiplier = 1;
                break;
            case B:// reset robot pose
                break;
            case LJS:// toggle headless
                robot.getChassis().switchHeadlessMode();
                break;
            case RJS:// reset robot pose
                robot.getNavigator().setMeasuredPose(0, 0, 0);
                robot.getNavigator().getChassis().driveMotors.StopAndResetEncoders();
                robot.getChassis().resetGyro();
                break;
            case RT:

                break;
            case LT:

                break;
            case RB:

                break;
            case LB:

                break;
            case Y:

                break;
        }
    }

    @Override
    public void ButtonHeld(int id, ControllerInput.Button button) {
        switch (button) {
            case RT:
                robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
                robot.armMotor.setPower(armPower);
                break ;
            case LT:
                robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
                robot.armMotor.setPower(-armPower);
                break ;
            case B:
                telemetry.addData("Are we there?: ", robot.getNavigator().goTowardsPose(-24, -10, 0, 0.4) ) ;
                break ;
            case Y:
                telemetry.addData("Are we there?: ", robot.getNavigator().goTowardsPose(0, 0, 0, 0.4) ) ;
                break ;
        }
    }

    @Override
    public void ButtonReleased(int id, ControllerInput.Button button) {
        switch (button) {
            case RT:  // Raise arm manually
                robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
                robot.armMotor.setPower(0) ;
                break ;
            case LT:  // Lower arm manually
                robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
                robot.armMotor.setPower(0) ;
                break ;
            case X:  // Toggle the gripper manually (open/close)
                robot.toggleGripper();
                break ;
            case DUP:  // Close the gripper and raise the arm to deliver high
                robot.closeGripper() ;
                robot.waitForTime(0.2) ;
                robot.armTarget = robot.armHigh ;
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
                break ;
            case DDOWN:
                robot.openGripper() ;
                robot.waitForTime(0.2) ;
                robot.armTarget = robot.armLow ;
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
                break ;
        }
    }





}
