package org.firstinspires.ftc.teamcode._RobotCode.TestBot;

import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ANG_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ANG_VEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_VEL_MOD;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;

@TeleOp(name = "*LIFT BOT TELEOP*", group = "Test")
@Config
public class LiftBotTeleop extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private LiftBot control;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns

    public static double autoSpeedModifier = 2; //used to change speed of automatic navigation

    public static double liftSpeed = 0.5;
    public static double intakeSpeed = 0.5;

    public static double turnP = 0.005;
    public static double turnI = 0.0;
    public static double turnD = 0.01;

    private double speedMultiplier = 1;

    private boolean busy = false;
    private double turnOffset = 0;

    public static int payloadControllerNumber = 1;

    private double intakeState = 0;
    private double liftHighPosition = 0 ;  // Specific to rotating arm - Notes the high (release) position of the arm

    @Override
    public void init() {
        control = new LiftBot(this, true, true, false);
        control.Init();

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
        if(control.isUSE_NAVIGATOR()){
            MAX_VEL_MOD  = autoSpeedModifier;
            MAX_ACCEL_MOD  = autoSpeedModifier;
            MAX_ANG_VEL_MOD  = autoSpeedModifier;
            MAX_ANG_ACCEL_MOD = autoSpeedModifier;
        }
    }

    @Override
    public void start(){control.Start();}

    @Override
    public void loop() {
        controllerInput1.Loop();
        controllerInput2.Loop();

        control.Update();

        //if robot isn't level, set speed to zero and exit loop
        /*if(!control.IsRobotLevel()){
            control.RawDrive(0,0,0);
            return;
        }*/

        if(!busy) {
            //Manage driving
            control.SetHeadingPID(turnP, turnI, turnD);
            control.DriveWithGamepad(controllerInput1, driveSpeed, turnSpeed, speedMultiplier);
        }
        //print telemetry
        if(control.isUSE_NAVIGATOR()) {
        }

        telemetry.addLine("*TELEOP DATA*");
        telemetry.addData("Speed Modifier", speedMultiplier);
        telemetry.addData("Payload Controller", payloadControllerNumber);
        telemetry.update();
    }

    ////DRIVING FUNCTIONS////


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
    }

    @Override
    public void AHeld(double controllerNumber) {
    }

    @Override
    public void BHeld(double controllerNumber) {
    }

    @Override
    public void XHeld(double controllerNumber) {
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
        if(controllerNumber == 1 && control.isUSE_PAYLOAD()){
            if(intakeState == 0) control.setIntakePower(intakeSpeed);
            else if (intakeState == 1) control.setIntakePower(0);
            else if (intakeState == 2) control.setIntakePower(-intakeSpeed);
            else if (intakeState == 3) control.setIntakePower(0);
            intakeState ++;
            if(intakeState > 3) intakeState = 0;
        }
    }

    @Override
    public void RBPressed(double controllerNumber) {
    }

    @Override
    public void LTPressed(double controllerNumber) {
    }

    @Override
    public void RTPressed(double controllerNumber) {
    }

    @Override
    public void LBHeld(double controllerNumber) {

    }

    @Override
    public void RBHeld(double controllerNumber) {

    }

    @Override
    public void LTHeld(double controllerNumber) {
        if(controllerNumber == 1 && control.isUSE_PAYLOAD()){
            control.setLiftPower(liftSpeed);
        }
    }

    @Override
    public void RTHeld(double controllerNumber) {
        if(controllerNumber == 1 && control.isUSE_PAYLOAD()){
            control.setLiftPower(-liftSpeed);
        }
    }

    @Override
    public void LBReleased(double controllerNumber) {

    }

    @Override
    public void RBReleased(double controllerNumber) {

    }

    @Override
    public void LTReleased(double controllerNumber) {
        if(controllerNumber == 1 && control.isUSE_PAYLOAD()){
            control.setLiftPower(0);
            //control.Arm().LockArm();
        }
    }

    @Override
    public void RTReleased(double controllerNumber) {
        if(controllerNumber == 1 && control.isUSE_PAYLOAD()){
            control.setLiftPower(0);
            //control.Arm().LockArm();
        }
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
        if(controllerNumber == payloadControllerNumber){
            if(controllerNumber == 1 && control.isUSE_PAYLOAD()){
                liftHighPosition = control.getLiftPosition() ;
                //control.Arm().SetSpinnerSpeed(0) ;
                telemetry.addData("Arm High Pos: ", liftHighPosition);
            }
        }
    }

    @Override
    public void DDownReleased(double controllerNumber) {

    }

    @Override
    public void DLeftReleased(double controllerNumber) {

    }

    @Override
    public void DRightReleased(double controllerNumber) {

    }

    @Override
    public void LJSPressed(double controllerNumber) {
        if(controllerNumber == 2) { //switch payload controllers at runtime
            if(payloadControllerNumber == 1) payloadControllerNumber = 2;
            else payloadControllerNumber = 1;
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
