package org.firstinspires.ftc.teamcode._RobotCode.Archived.TankDriveBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.TankChassisControl;

@TeleOp(name = "TankDriveTeleop", group = "ALL")
@Config
@Disabled
public class TankDriveTeleop extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private TankChassisControl control;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = -1;//used to change how fast robot drives
    public static double spotTurnSpeed = -1;//used to change how fast robot turns
    public static double sweepTurnSpeed = -0.4;//used to change how fast the robot turns in headless while driving

    //heading pid controller configuration
    public static double turnP = 0.005;
    public static double turnI = 0.0;
    public static double turnD = 0.01;
    public static boolean reversePID = true;

    private double speedMultiplier = 1;//overall speed multiplier

    private boolean busy = false;
    private double turnOffset = 0;

    public static int payloadControllerNumber = 1;

    @Override
    public void init() {
        control = new TankChassisControl(this, new HermesLog("TankChassis", 500, this), true, false, false);

        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();
    }

    @Override
    public void start(){control.StartCoreRobotModules();}

    @Override
    public void loop() {
        controllerInput1.Loop();
        controllerInput2.Loop();

        control.Update();

        if(!busy) {
            //Manage driving
            control.SetHeadingPID(turnP, turnI, turnD, reversePID);

            double speedNormal = controllerInput1.GetLJSY()*driveSpeed*speedMultiplier;
            double speedHeadless = Math.abs(controllerInput1.CalculateLJSMag()*driveSpeed*speedMultiplier);
            telemetry.addData("Speed Headless", speedHeadless);
            double targetHeading = controllerInput1.CalculateLJSAngle()-180;
            double turnSpeed = controllerInput1.GetRJSX()*speedMultiplier;
            double sweepTurn = controllerInput1.GetLJSX()*sweepTurnSpeed*speedMultiplier;

            control.Drive(speedNormal, speedHeadless, targetHeading, turnSpeed*spotTurnSpeed, turnSpeed*sweepTurnSpeed);

        }
        //print telemetry
        if(control.isUSE_NAVIGATOR()) {
           /* control.GetOrion().PrintVuforiaTelemetry(0);
            control.GetOrion().PrintTensorflowTelemetry();*/
        }

        telemetry.addLine("*TELEOP DATA*");
        telemetry.addData("Speed Modifier", speedMultiplier);
        telemetry.addData("Payload Controller", payloadControllerNumber);

        telemetry.update();
    }

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
    public void BReleased(double controllerNumber) {

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

    }

    @Override
    public void RTReleased(double controllerNumber) {
        if(controllerNumber == 1) speedMultiplier = 1;
    }

    @Override
    public void DUpPressed(double controllerNumber) {

    }

    @Override
    public void DDownPressed(double controllerNumber) {

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
        if(controllerNumber == 2) { //switch payload controllers at runtime
            if(payloadControllerNumber == 1) payloadControllerNumber = 2;
            else payloadControllerNumber = 1;
        }
    }

    @Override
    public void RJSPressed(double controllerNumber) {
        //if(controllerNumber == 1) control.SwitchHeadlessMode();
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
