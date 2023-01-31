package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;

@TeleOp(name = "*JUAN Gripper Calibrate - RELEASED*", group = "JUAN")
@Config
public class JuanGripperCalibrate_RELEASED extends OpMode implements ControllerInputListener
{
    private static final String VERSION = "1.13";

    ////Dependencies////
    private Juan_RELEASED robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns
    private final double speedMultiplier = 1;

    private final double liftOverrideSpeed = 5;

    public static int payloadControllerNumber = 1;

    public static double gripperPosition;

    private JuanPayload_RELEASED.GripperController gripper;

    @Override
    public void init() {
        robot = new Juan_RELEASED(this,true,true,false);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        gripper = robot.getPayload().getGripper();

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
        telemetry.addData("Version", VERSION);

        controllerInput1.Loop();
        controllerInput2.Loop();

        telemetry.addData("Servo Position", gripperPosition);

        gripper.getServo().setPosition(gripperPosition);

        //update robot
        robot.update();
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

        if(robot.USE_PAYLOAD)robot.getPayload().printTelemetry();
    }

    @Override
    public void stop(){
        robot.stop();
    }

    ////INPUT MAPPING////
    @Override
    public void ButtonPressed(int id, Button button) {

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
