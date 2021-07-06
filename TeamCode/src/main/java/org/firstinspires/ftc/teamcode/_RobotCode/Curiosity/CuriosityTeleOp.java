package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Input.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.Input.ControllerInputListener;

import static org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive.DriveConstants.MAX_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive.DriveConstants.MAX_ANG_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive.DriveConstants.MAX_ANG_VEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive.DriveConstants.MAX_VEL_MOD;

@TeleOp(name = "*CURIOSITY TELEOP*", group = "Curiosity")
@Config
public class CuriosityTeleOp extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private CuriosityUltimateGoalControl control;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns

    public static double autoSpeedModifier = 2; //used to change speed of automatic navigation

    public static double turnP = 0.005;
    public static double turnI = 0.0;
    public static double turnD = 0.01;

    public static double shootX = 75.0;
    public static double shootY = -10.0;
    public static double shootH = 0.0;
    public static double shootBOffset = 0.0;

    private double speedMultiplier = 1;

    private boolean busy = false;
    private double turnOffset = 0;

    public static int payloadControllerNumber = 1;

    @Override
    public void init() {
        control = new CuriosityUltimateGoalControl(this, true, true, true);
        control.Init();

        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        if(control.isUSE_PAYLOAD()) control.StarpathToIntake();

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

        if(!busy) {
            //Manage driving
            //if(control.isUSE_NAVIGATOR()) ManageDrivingRoadrunner();
            control.SetDrivePID(turnP, turnI, turnD);
            ManageDriveMovementCustom();

            control.IntakeLoop(); //loop the intake
        }
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
            telemetry.addData("Moving at ", controllerInput1.CalculateLJSAngle());
        }
        //TURN if right joystick magnitude > 0.1 and not moving
        else if (Math.abs(controllerInput1.GetRJSX()) > 0.1) {
            control.RawTurn(controllerInput1.GetRJSX() * turnSpeed * speedMultiplier);//turns at speed according to rjs1
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
        /*if(controllerNumber == payloadControllerNumber){
            control.ModifyForPowerShot();
        }*/
        if(controllerNumber == 1) control.ResetGyro();
    }

    @Override
    public void XPressed(double controllerNumber) {
        if(controllerNumber == 1) control.SetOriginToVumark(0);
        if(controllerNumber == 2) control.Powershot();
    }

    @Override
    public void YPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber) {
            control.SetOriginToVumark(0);
            control.ShootThree();
        }
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
        if(controllerNumber == payloadControllerNumber) control.StopModifyForPowerShot();
    }

    @Override
    public void XReleased(double controllerNumber) {
    }

    @Override
    public void YReleased(double controllerNumber) {
        control.StopShootThree();
    }

    @Override
    public void LBPressed(double controllerNumber) {
        if(controllerNumber == 1) control.ToggleIntaking();
    }

    @Override
    public void RBPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber) control.ToggleShooterMotors();
    }

    @Override
    public void LTPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber) control.ReverseIntake();
    }

    @Override
    public void RTPressed(double controllerNumber) {
        if(controllerNumber == 1) speedMultiplier = 0.25;
    }

    @Override
    public void LBHeld(double controllerNumber) {

    }

    @Override
    public void RBHeld(double controllerNumber) {

    }

    @Override
    public void LTHeld(double controllerNumber) {

    }

    @Override
    public void RTHeld(double controllerNumber) {

    }

    @Override
    public void LBReleased(double controllerNumber) {

    }

    @Override
    public void RBReleased(double controllerNumber) {
    }

    @Override
    public void LTReleased(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber) control.IntakeOn();
    }

    @Override
    public void RTReleased(double controllerNumber) {
        if(controllerNumber == 1) speedMultiplier = 1;
    }

    @Override
    public void DUpPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber){
            control.RotateStarpathToNextPos();
        }
    }

    @Override
    public void DDownPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber){
            control.RotateStarpathToPreviousPos();
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
        //if(controllerNumber == 1) control.GoToHome();
        if(controllerNumber == 1) {
            control.GetOrion().MoveLine(shootX, shootY, shootH);
            control.ShooterOn();
            control.GetOrion().Turn(control.GetOrion().GetVuforiaBearing(0)+shootBOffset);
            control.SetOriginToVumark(0);

        }
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
