package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;


@TeleOp(name = "*INGENUITY TELEOP*", group = "ingenuity")
@Config
public class IngenuityTeleop extends OpMode implements ControllerInputListener {
    ////Dependencies////
    private IngenuityPowerPlayBot robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;
    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns

    private double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;

    public static double armPower = 0.5;

    private double armStartPos = 0.3;
    private int armSetpointIdx = 0;
    private double[] armStops = {0.0, 0.1355, 0.23177, 0.3476};

    @Override
    public void init() {
        robot = new IngenuityPowerPlayBot(this, true, true, true, armStartPos);
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
    public void start() {
        robot.start();
        robot.getChassis().resetGyro();
        //if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE) robot.SetInputOffset(90); //90 is blue, -90 is red
        //else if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.RED) robot.SetInputOffset(-90); //90 is blue, -90 is red
        robot.getChassis().setHeadlessMode(false);
    }

    @Override
    public void loop() {
        controllerInput1.loop();
        controllerInput2.loop();
        //navigator kill switch
        if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {

        }
        //update robot
        robot.update();
        //manage driving
        robot.getChassis().driveWithGamepad(controllerInput1, speedMultiplier);
        //telemetry
        robot.readSignal();
        printTelemetry();
        telemetry.update();
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
        telemetry.addData("Distance: ", robot.sensorDistance.getDistance(DistanceUnit.MM));

        telemetry.addData("Gripper:  ", robot.servoTarget);
        telemetry.addData("Arm:      ", robot.getPayload().getArm().getPosition());
        //Dead wheel positions
        telemetry.addLine("Dead wheel positions");
        double[] deadWheelPositions = robot.getNavigator().getDeadWheelPositions();
        telemetry.addData("LEFT dead wheel:       ", deadWheelPositions[0] + " inches");
        telemetry.addData("RIGHT dead wheel:      ", deadWheelPositions[1] + " inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2] + " inches");
        //Odometry estimated pose
        telemetry.addLine();
        telemetry.addLine("Robot pose");
        Pose2d robotPose = robot.getNavigator().getMeasuredPose();
        telemetry.addData("X, Y, Angle", robotPose.getX() + ", " + robotPose.getY() + ", " + Math.toDegrees(robotPose.getHeading()));
        telemetry.addLine();
    }

    @Override
    public void stop() {
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
                robot.getNavigator().getChassis().driveMotors.stopAndResetEncoders();
                robot.getChassis().resetGyro();
                break;
            case RT:

                break;
            case LT:

                break;
            case RB:
                if (this.armSetpointIdx < 3) {
                    armSetpointIdx += 1;
                    robot.getPayload().getArm().goToPosition(armStops[armSetpointIdx] - armStartPos);
                }
                break;
            case LB:
                if (this.armSetpointIdx > 0) {
                    armSetpointIdx -= 1;
                    robot.getPayload().getArm().goToPosition(armStops[armSetpointIdx] - armStartPos);
                }
                break;
            case Y:

                break;
        }
    }

    @Override
    public void ButtonHeld(int id, ControllerInput.Button button) {
        switch (button) {
            case RT:
                robot.getPayload().getArm().setPowerRaw(armPower);
                break;
            case LT:
                robot.getPayload().getArm().setPowerRaw(-armPower);
                break;

        }
    }

    @Override
    public void ButtonReleased(int id, ControllerInput.Button button) {
        switch (button) {
            case RT:
            case LT:
                robot.getPayload().getArm().setPowerRaw(0);
                break;
            case X:
                robot.toggleGripper();
                //robot.gripperServo.setPosition(robot.servoTarget) ;
                break;
        }
    }


}
