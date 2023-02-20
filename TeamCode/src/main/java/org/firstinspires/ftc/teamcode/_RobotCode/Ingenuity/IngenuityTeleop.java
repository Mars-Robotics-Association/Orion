package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.opencv.core.Mat;

import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Iterator;
import java.util.Queue;

import kotlin.jvm.internal.PropertyReference0Impl;

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

    public static double TURBO_SPEED = 1.0;
    public static double STANDARD_SPEED = 0.5;

    private double speedMultiplier = STANDARD_SPEED;

    public static int payloadControllerNumber = 1;

    public static double armPower = 0.5;

    private ArrayDeque<Pose2d> autoNavQueue;

    private double matSegmentLength;
    private double matSegmentWidth;


    @Override
    public void init() {
        robot = new IngenuityPowerPlayBot(this, true, true, true);
        robot.resetArmHomePosition(0);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        autoNavQueue = new ArrayDeque<Pose2d>();

        //hardwareMap.dcMotor.get("FR").setDirection(DcMotorSimple.Direction.REVERSE);
        //hardwareMap.dcMotor.get("FL").setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        msStuckDetectLoop = 5000;

        matSegmentLength = 21;
        matSegmentWidth = 24;
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
        if (autoNavQueue.isEmpty() || !robot.getChassis().getIsHeadless()) {
            robot.getChassis().driveWithGamepad(controllerInput1, speedMultiplier);
        } else if (!autoNavQueue.isEmpty()) {
            Pose2d curr = robot.navigator.getMeasuredPose();
            Pose2d next = autoNavQueue.peek();
            if (Math.sqrt(Math.pow(next.getX() - curr.getX(), 2) + Math.pow(next.getY() - curr.getY(), 2)) < 2.5) {
                autoNavQueue.remove();
                next = autoNavQueue.peek();
            }
            if (next != null) {
                robot.navigator.goTowardsPose(next.getX(), next.getY(), next.getHeading(), 0.8);
            }
        }
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
        telemetry.addData("X, Y, Angle",
                Math.round(robotPose.getX() * 100) / 100
                        + ", " + Math.round(robotPose.getY() * 100) / 100 + ", "
                        + Math.round(Math.toDegrees(robotPose.getHeading()) * 100) / 100);
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
//                if (speedMultiplier == .TURBO_SPEED) speedMultiplier = STANDARD_SPEED;
//                else speedMultiplier = .TURBO_SPEED;
                break;
            case B:// reset robot pose
                break;
            case LJS:// toggle headless
                if (robot.getChassis().getIsHeadless()) {
                    autoNavQueue.clear();
                }
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
                robot.raiseArmToNextStop();
                break;
            case LB:
                robot.lowerArmToNextStop();
                break;
            case Y:

                break;
        }
    }

    @Override
    public void ButtonHeld(int id, ControllerInput.Button button) {
        switch (button) {
            case A:
                speedMultiplier = TURBO_SPEED;
                break;
            case RT:
                robot.getPayload().getArm().setPowerRaw(armPower);
                robot.resetArmStateMachine(); // use of the trigger resets arm position state
                break;
            case LT:
                robot.getPayload().getArm().setPowerRaw(-armPower);
                robot.resetArmStateMachine();
                break;

        }
    }

    @Override
    public void ButtonReleased(int id, ControllerInput.Button button) {
        switch (button) {
            case A:
                speedMultiplier = STANDARD_SPEED;
                break;
            case RT:
            case LT:
                robot.getPayload().getArm().setPowerRaw(0);
                break;
            case X:
                robot.toggleGripper();
                //robot.gripperServo.setPosition(robot.servoTarget) ;
                break;
            case Y:
                robot.resetArmHomePosition(0);
                break;
            case DUP:
                if (robot.getChassis().getIsHeadless()) {
                    Pose2d startPose;
                    if (autoNavQueue.isEmpty()) {
                        startPose = robot.navigator.getMeasuredPose();
                    } else {
                        startPose = autoNavQueue.peekLast();
                    }
                    autoNavQueue.add(new Pose2d(startPose.getX() + matSegmentLength, startPose.getY(), startPose.getHeading()));
                }
                break;
            case DLEFT:
                if (robot.getChassis().getIsHeadless()) {
                    Pose2d startPose;
                    if (autoNavQueue.isEmpty()) {
                        startPose = robot.navigator.getMeasuredPose();
                    } else {
                        startPose = autoNavQueue.peekLast();
                    }
                    autoNavQueue.add(new Pose2d(startPose.getX(), startPose.getY() - matSegmentWidth, startPose.getHeading()));
                }
                break;
            case DRIGHT:
                if (robot.getChassis().getIsHeadless()) {
                    Pose2d startPose;
                    if (autoNavQueue.isEmpty()) {
                        startPose = robot.navigator.getMeasuredPose();
                    } else {
                        startPose = autoNavQueue.peekLast();
                    }
                    autoNavQueue.add(new Pose2d(startPose.getX(), startPose.getY() + matSegmentWidth, startPose.getHeading()));
                }
                break;
            case DDOWN:
                if (robot.getChassis().getIsHeadless()) {
                    Pose2d startPose;
                    if (autoNavQueue.isEmpty()) {
                        startPose = robot.navigator.getMeasuredPose();
                    } else {
                        startPose = autoNavQueue.peekLast();
                    }
                    autoNavQueue.add(new Pose2d(startPose.getX() - matSegmentLength, startPose.getY(), startPose.getHeading()));
                }
                break;
        }
    }


}
