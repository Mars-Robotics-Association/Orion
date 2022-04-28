package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;


@TeleOp(name = "*ERASMUS TELEOP*", group = "Erasmus")
@Config
public class ErasmusTeleop extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private ErasmusRobot robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns

    public static double turnP = 0.005;
    public static double turnI = 0.0;
    public static double turnD = 0.01;

    private double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;



    @Override
    public void init() {
        robot = new ErasmusRobot(this, true, true, false);
        robot.Init();

        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        //hardwareMap.dcMotor.get("FR").setDirection(DcMotorSimple.Direction.REVERSE);
        //hardwareMap.dcMotor.get("FL").setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();


        msStuckDetectLoop = 15000;


        //set roadrunner speed modifiers
        if(robot.USE_NAVIGATOR){
        }
    }

    @Override
    public void start(){
        robot.Start();
        robot.chassis.ResetGyro();
        //if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE) robot.SetInputOffset(90); //90 is blue, -90 is red
        //else if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.RED) robot.SetInputOffset(-90); //90 is blue, -90 is red
        robot.chassis.SetHeadlessMode(true);
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        controllerInput2.Loop();

        //KILL SWITCH FOR NAVIGATOR
        if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {
            //robot.navigation.StopNavigator();
            if(robot.USE_PAYLOAD) robot.TurretArm().StopArmThread();
            if(robot.USE_PAYLOAD) robot.blinkinController.Lime();
        }

        //if(robot.navigation.IsThreadRunning()) return;

        if(robot.fieldSide == BaseRobot.FieldSide.BLUE){
            telemetry.addData("Alliance Side", "BLUE");
            if(robot.USE_PAYLOAD) robot.blinkinController.Blue();
        }
        else {
            telemetry.addData("Alliance Side", "RED");
            if(robot.USE_PAYLOAD) robot.blinkinController.Red();
        }

        robot.Update();

        //robot.navigation.PrintSensorTelemetry();

        //Manage driving
        robot.chassis.SetHeadingPID(turnP, turnI, turnD);
        robot.chassis.DriveWithGamepad(controllerInput1, driveSpeed, turnSpeed, speedMultiplier);

        //print telemetry
        if(robot.USE_NAVIGATOR) {
            //control.GetOrion().PrintVuforiaTelemetry(0);
            //control.GetOrion().PrintTensorflowTelemetry();
        }

        telemetry.update();
    }

    @Override
    public void stop(){
        robot.Stop();
    }

    ////INPUT MAPPING////
    @Override
    public void ButtonPressed(int id, ControllerInput.Button button) {
        //controller 1
        if(id == 1){
            //Colored buttons
            if(button == Button.A){
                if (speedMultiplier == 1) speedMultiplier = 0.5;
                else speedMultiplier = 1; }
            if(button == Button.B);
            if(button == Button.X && robot.USE_PAYLOAD) robot.TurretArm().ReturnToHomeAndIntake(); //TODO: revert to resetAndIntake()
            if(button == Button.Y && robot.USE_PAYLOAD) robot.TurretArm().CycleIntakeState(1);
            //Bumpers
            if(button == Button.LB);
            if(button == Button.RB);
            //Triggers
            if(button == Button.LT);
            if(button == Button.RT);
            //Dpad
            if(button == Button.DUP && robot.USE_PAYLOAD){
                robot.TurretArm().AutoIntakeTierUp();
                robot.TurretArm().GoToAutoTier();
            }
            if(button == Button.DDOWN && robot.USE_PAYLOAD){
                robot.TurretArm().AutoIntakeTierDown();
                robot.TurretArm().GoToAutoTier();
            }
            if(button == Button.DLEFT);
            if(button == Button.DRIGHT);
            //Joystick Buttons
            if(button == Button.LJS);
            if(button == Button.RJS && robot.USE_CHASSIS) robot.chassis.ResetGyro();
        }
        //controller 2
        if(id == 2){
            //Colored buttons
            if(button == Button.A);
            if(button == Button.B);
            if(button == Button.X);
            if(button == Button.Y);
            //Bumpers
            if(button == Button.LB);
            if(button == Button.RB);
            //Triggers
            if(button == Button.LT);
            if(button == Button.RT);
            //Dpad
            if(button == Button.DUP);
            if(button == Button.DDOWN);
            if(button == Button.DLEFT);
            if(button == Button.DRIGHT);
            //Joystick Buttons
            if(button == Button.LJS);
            if(button == Button.RJS && robot.USE_CHASSIS) robot.Arm().ResetToZero();
        }
    }

    @Override
    public void ButtonHeld(int id, ControllerInput.Button button) {
        //controller 1
        if(id == 1){
            //Colored buttons
            if(button == Button.A);
            if(button == Button.B);
            if(button == Button.X);
            if(button == Button.Y);
            //Bumpers
            if(button == Button.LB && robot.USE_PAYLOAD) robot.Turret().SetPowerClamped(-1);
            if(button == Button.RB && robot.USE_PAYLOAD) robot.Turret().SetPowerClamped(1);
            //Triggers
            if(button == Button.LT && robot.USE_PAYLOAD) robot.Arm().SetPowerClamped(1);
            if(button == Button.RT && robot.USE_PAYLOAD) robot.Arm().SetPowerClamped(-1);
            //Dpad
            if(button == Button.DUP);
            if(button == Button.DDOWN);
            if(button == Button.DLEFT);
            if(button == Button.DRIGHT);
            //Joystick Buttons
            if(button == Button.LJS);
            if(button == Button.RJS);
        }
        //controller 2
        if(id == 2){
            //Colored buttons
            if(button == Button.A);
            if(button == Button.B);
            if(button == Button.X);
            if(button == Button.Y);
            //Bumpers
            if(button == Button.LB);
            if(button == Button.RB);
            //Triggers
            if(button == Button.LT && robot.USE_PAYLOAD) robot.Arm().SetPowerRaw(1);
            if(button == Button.RT && robot.USE_PAYLOAD) robot.Arm().SetPowerRaw(-1);
            //Dpad
            if(button == Button.DUP);
            if(button == Button.DDOWN);
            if(button == Button.DLEFT);
            if(button == Button.DRIGHT);
            //Joystick Buttons
            if(button == Button.LJS);
            if(button == Button.RJS);
        }
    }

    @Override
    public void ButtonReleased(int id, ControllerInput.Button button) {
        //controller 1
        if(id == 1){
            //Colored buttons
            if(button == Button.A);
            if(button == Button.B);
            if(button == Button.X);
            if(button == Button.Y);
            //Bumpers
            if(button == Button.LB && robot.USE_PAYLOAD) robot.Turret().SetPowerClamped(0);
            if(button == Button.RB && robot.USE_PAYLOAD) robot.Turret().SetPowerClamped(0);
            //Triggers
            if(button == Button.LT && robot.USE_PAYLOAD) robot.Arm().SetPowerRaw(0);
            if(button == Button.RT && robot.USE_PAYLOAD) robot.Arm().SetPowerRaw(0);
            //Dpad
            if(button == Button.DUP);
            if(button == Button.DDOWN);
            if(button == Button.DLEFT);
            if(button == Button.DRIGHT);
            //Joystick Buttons
            if(button == Button.LJS);
            if(button == Button.RJS);
        }
        //controller 2
        if(id == 2){
            //Colored buttons
            if(button == Button.A);
            if(button == Button.B);
            if(button == Button.X);
            if(button == Button.Y);
            //Bumpers
            if(button == Button.LB);
            if(button == Button.RB);
            //Triggers
            if(button == Button.LT && robot.USE_PAYLOAD) robot.Arm().SetPowerRaw(0);
            if(button == Button.RT && robot.USE_PAYLOAD) robot.Arm().SetPowerRaw(0);
            //Dpad
            if(button == Button.DUP);
            if(button == Button.DDOWN);
            if(button == Button.DLEFT);
            if(button == Button.DRIGHT);
            //Joystick Buttons
            if(button == Button.LJS);
            if(button == Button.RJS);
        }
    }


}
