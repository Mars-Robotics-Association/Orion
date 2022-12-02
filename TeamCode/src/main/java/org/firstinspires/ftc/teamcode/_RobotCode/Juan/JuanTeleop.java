package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@TeleOp(name = "*JUAN TELEOP*", group = "JUAN")
@Config
public class JuanTeleop extends OpMode implements ControllerInputListener
{
    private static final String VERSION = "1.14";

    ////Dependencies////
    private Juan robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns
    private final double speedMultiplier = 1;

    private final double liftOverrideSpeed = 5;

    public static int payloadControllerNumber = 1;

    @Override
    public void init() {
        robot = new Juan(this,true,true,false);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();
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

        MecanumChassis chassis = robot.getChassis();

        controllerInput1.Loop();
        controllerInput2.Loop();

        //update robot
        robot.update();
        //manage driving
        if(gamepad2.atRest()){
            chassis.driveWithGamepad(controllerInput1, driveSpeed, turnSpeed, 1);
        }else{
            chassis.driveWithGamepad(controllerInput2, driveSpeed, turnSpeed, 1);
        }

        telemetry.addData("Gripper Position", robot.getPayload().getGripper().getPosition());

        int direction = 0;
        if(gamepad1.dpad_up || gamepad2.dpad_up)direction++;
        if(gamepad1.dpad_down || gamepad2.dpad_down)direction++;
        robot.getPayload().getLift().setSpeed(direction * liftOverrideSpeed);

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
        telemetry.addData("Toggle headless mode: ", "Press LOGITECH");

        if(robot.USE_PAYLOAD)robot.getPayload().printTelemetry();
    }

    @Override
    public void stop(){
        robot.stop();
    }

    ////INPUT MAPPING////
    @Override
    public void ButtonPressed(int id, Button button) {
        if(!robot.USE_PAYLOAD)return;
            JuanPayload payload = robot.getPayload();
            JuanPayload.LiftController lift = payload.getLift();
            JuanPayload.GripperController gripper = payload.getGripper();

            switch (button) {
                case LB:
                    gripper.grab();
                    break;
                case RB:
                    gripper.release();
                    break;
                case LT:
                    gripper.grabMore();
                    break;
                case RT:
                    gripper.releaseMore();
                    break;
                case GUIDE:
                    lift.reset();
                    break;
                case A:
                    lift.goToPreset(JuanPayload.PresetHeight.BOTTOM);
                    break;
                case B:
                    lift.goToPreset(JuanPayload.PresetHeight.LOW);
                    break;
                case Y:
                    lift.goToPreset(JuanPayload.PresetHeight.MEDIUM);
                    break;
                case X:
                    lift.goToPreset(JuanPayload.PresetHeight.HIGH);
                    break;
                case DUP:
                    lift.setSpeed(liftOverrideSpeed);
                    break;
                case DDOWN:
                    lift.setSpeed(-liftOverrideSpeed);
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
