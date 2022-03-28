package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ANG_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ANG_VEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_VEL_MOD;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;

import java.util.Arrays;

@TeleOp(name = "*INGENUITY TELEOP*", group = "Ingenuity")
@Config
public class IngenuityTeleop extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private IngenuityControl control;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns

    public static double deadZone = 0.1;

    public static double autoSpeedModifier = 2; //used to change speed of automatic navigation

    public static double turnP = 0.005;
    public static double turnI = 0.0;
    public static double turnD = 0.01;

    public static double speedMultiplier = 1;

    private boolean busy = false;
    private double turnOffset = 0;

    public static int payloadControllerNumber = 1;

    @Override
    public void init() {
        control = new IngenuityControl(this, true, true, false);
        control.Init();

        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        //reversing the motors so that it is easier to drive
        hardwareMap.dcMotor.get("FR").setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.dcMotor.get("RR").setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.dcMotor.get("FL").setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.dcMotor.get("RL").setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();


        msStuckDetectLoop = 15000;

        //set roadrunner speed modifiers
        if(control.isUSE_NAVIGATOR()){
            MAX_VEL_MOD  = autoSpeedModifier;
            MAX_ACCEL_MOD  = autoSpeedModifier;
            MAX_ANG_VEL_MOD  = autoSpeedModifier;
            MAX_ANG_ACCEL_MOD = autoSpeedModifier;
        }

        control.InitCoreRobotModules();
    }

    @Override
    public void start(){
        control.Start();
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        controllerInput2.Loop();

        control.Update();

        //Manage driving
        control.SetHeadingPID(turnP, turnI, turnD);
        ManageDriveMovementCustom();

        //print telemetry
        if(control.isUSE_NAVIGATOR()) {
            control.GetOrion().PrintVuforiaTelemetry(0);
            control.GetOrion().PrintTensorflowTelemetry();
        }

        telemetry.addLine("*TELEOP DATA*");
        telemetry.addData("Speed Modifier", speedMultiplier);
        telemetry.addData("Payload Controller", payloadControllerNumber);

        telemetry.update();
    }

    ////DRIVING FUNCTIONS////

    private void ManageDrivingRoadrunner() {
        double moveX = -gamepad1.left_stick_y*driveSpeed*speedMultiplier;
        double moveY = -gamepad1.left_stick_x*driveSpeed*speedMultiplier;
        double turn = -gamepad1.right_stick_x*turnSpeed*speedMultiplier + turnOffset;
        control.GetOrion().MoveRaw(moveX, moveY, turn);
    }

    private void ManageDriveMovementCustom() {
        //MOVE if left joystick magnitude > 0.1
        if (controllerInput1.CalculateLJSMag() > 0.1) {
            control.RawDrive(controllerInput1.CalculateLJSAngle(), controllerInput1.CalculateLJSMag() * driveSpeed * speedMultiplier, controllerInput1.GetRJSX() * turnSpeed * speedMultiplier);//drives at (angle, speed, turnOffset)
            double[] params = {controllerInput1.CalculateLJSAngle(), controllerInput1.CalculateLJSMag() * driveSpeed * speedMultiplier, controllerInput1.GetRJSX() * turnSpeed * speedMultiplier};
            telemetry.addData("Passed Parameters: ", Arrays.toString(params));
            telemetry.addData("Moving at ", controllerInput1.CalculateLJSAngle());
        }
        //TURN if right joystick magnitude > 0.1 and not moving
        else if (Math.abs(controllerInput1.GetRJSX()) > 0.1) {
            control.RawTurn(controllerInput1.GetRJSX() * turnSpeed * speedMultiplier);//turns at speed according to rjs1
            double[] params = {controllerInput1.GetRJSX() * turnSpeed * speedMultiplier};
            telemetry.addData("Passed Parameters: ", Arrays.toString(params));
            telemetry.addData("Turning", true);
        }
        else {
            control.GetChassis().SetMotorSpeeds(0,0,0,0);
        }
    }

    ////INPUT MAPPING////

    @Override
    public void APressed(double controllerNumber) {
        if(controllerNumber == 1) {
            if (speedMultiplier == 1) speedMultiplier = 0.5;
            else speedMultiplier = 1;
        }
    }

    @Override
    public void BPressed(double controllerNumber) {
        if(controllerNumber == 1) control.ResetGyro();
    }

    @Override
    public void XPressed(double controllerNumber) {

    }

    @Override
    public void YPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber) {

        }
    }

    @Override
    public void AHeld(double controllerNumber) {

    }

    @Override
    public void BHeld(double controllerNumber) {
        control.GetDuck().RedSide();

    }

    @Override
    public void XHeld(double controllerNumber) {
        control.GetDuck().BlueSide();
    }

    @Override
    public void YHeld(double controllerNumber) {
    }

    @Override
    public void AReleased(double controllerNumber) {

    }

    @Override
    public void BReleased(double controllerNumber)  {
    }

    @Override
    public void XReleased(double controllerNumber) {
    }

    @Override
    public void YReleased(double controllerNumber) {

    }

    @Override
    public void LBPressed(double controllerNumber) {
    }

    @Override
    public void RBPressed(double controllerNumber) {
    }

    @Override
    public void LTPressed(double controllerNumber) {
    }

    @Override
    public void RTPressed(double controllerNumber) {
        //if(controllerNumber == 1) speedMultiplier = 0.25;
    }

    @Override
    public void LBHeld(double controllerNumber) {
        control.GetIntake().on(1);
    }

    @Override
    public void RBHeld(double controllerNumber) {
        control.GetIntake().on(0);
    }

    @Override
    public void LTHeld(double controllerNumber) {
        control.GetLift().SetPowerRaw(-0.2);
    }

    @Override
    public void RTHeld(double controllerNumber) {
        control.GetLift().SetPowerRaw(0.2);
    }

    @Override
    public void LBReleased(double controllerNumber) {
        control.GetIntake().off();
    }

    @Override
    public void RBReleased(double controllerNumber) { control.GetIntake().off(); }

    @Override
    public void LTReleased(double controllerNumber) {
        control.GetLift().Lock();
    }

    @Override
    public void RTReleased(double controllerNumber) {
        control.GetLift().Lock();
    }

    @Override
    public void DUpPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber){
        }
    }

    @Override
    public void DDownPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber){
        }
        control.RawDrive(0, 0, 0);
    }

    @Override
    public void DLeftPressed(double controllerNumber) {

    }

    @Override
    public void DRightPressed(double controllerNumber) {

    }

    @Override
    public void DUpHeld(double controllerNumber) {

    }

    @Override
    public void DDownHeld(double controllerNumber) {

    }

    @Override
    public void DLeftHeld(double controllerNumber) {

    }

    @Override
    public void DRightHeld(double controllerNumber) {

    }

    @Override
    public void DUpReleased(double controllerNumber) {

    }

    @Override
    public void DDownReleased(double controllerNumber) {

    }

    @Override
    public void DLeftReleased(double controllerNumber) {
        control.GetDuck().Stop();
    }

    @Override
    public void DRightReleased(double controllerNumber) {
        control.GetDuck().Stop();
    }

    @Override
    public void LJSPressed(double controllerNumber) {
        if(controllerNumber == 1) { //switch payload controllers at runtime
            control.ResetGyro();
        }
    }

    @Override
    public void RJSPressed(double controllerNumber) {
        if(controllerNumber == 1) control.SwitchHeadlessMode();
        //if(controllerNumber == 1) control.TurnToZero();
    }

    @Override
    public void LJSHeld(double controllerNumber) {

    }

    @Override
    public void RJSHeld(double controllerNumber) {

    }

    @Override
    public void LJSReleased(double controllerNumber) {

    }

    @Override
    public void RJSReleased(double controllerNumber) {

    }
}
