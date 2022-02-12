package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavigation;

import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ANG_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_ANG_VEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.DriveConstants.MAX_VEL_MOD;

@TeleOp(name = "*CURIOSITY TELEOP*", group = "Curiosity")
@Config
public class CuriosityTeleop extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private CuriosityRobot control;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns

    public static double autoSpeedModifier = 2; //used to change speed of automatic navigation

    public static double armSpeed = 1;
    public static double intakeSpeed = 1;

    public static double turnP = 0.005;
    public static double turnI = 0.0;
    public static double turnD = 0.01;

    private double speedMultiplier = 1;

    private boolean busy = false;
    private double turnOffset = 0;

    public static int payloadControllerNumber = 1;



    @Override
    public void init() {
        control = new CuriosityRobot(this, true, true, true);
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
    public void start(){
        control.Start();
        control.ResetGyro();
        //control.GetImu().OffsetGyro(-90);//apply offset to robot's gyro at start of match
        if(control.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE) control.SetInputOffset(90); //90 is blue, -90 is red
        else if(control.navigation.side == FreightFrenzyNavigation.AllianceSide.RED) control.SetInputOffset(-90); //90 is blue, -90 is red
        //control.navigation
        control.SetHeadlessMode(true);
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        controllerInput2.Loop();

        //KILL SWITCH FOR NAVIGATOR
        if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {
            control.navigation.StopNavigator();
            control.TurretArm().StopAutoLeveller();
            control.blinkinController.Lime();
        }

        if(control.navigation.IsThreadRunning()) return;

        if(control.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE){
            telemetry.addData("Alliance Side", "BLUE");
            control.blinkinController.Blue();
        }
        else {
            telemetry.addData("Alliance Side", "RED");
            control.blinkinController.Red();
        }

        control.Update();

        control.navigation.PrintSensorTelemetry();

        //Manage driving
        control.SetHeadingPID(turnP, turnI, turnD);
        control.DriveWithGamepad(controllerInput1, driveSpeed, turnSpeed, speedMultiplier);

        //print telemetry
        if(control.isUSE_NAVIGATOR()) {
            //control.GetOrion().PrintVuforiaTelemetry(0);
            //control.GetOrion().PrintTensorflowTelemetry();
        }

        telemetry.update();
    }

    @Override
    public void stop(){
        control.Stop();
    }

    ////INPUT MAPPING////

    @Override
    public void APressed(double controllerNumber) {
        if(controllerNumber == 1) {
            if (speedMultiplier == 1) speedMultiplier = 0.5;
            else speedMultiplier = 1;
        }
        else if(controllerNumber == 2){
            control.navigation.StartCollecting(control.turretArm.GetCurrentAutoTierRotation());
        }
    }

    @Override
    public void BPressed(double controllerNumber) {

        if(controllerNumber == 1) {
            //control.navigation.CollectFreightLinear();
            control.navigation.StarGoToCollect();
        }

    }

    @Override
    public void XPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber && control.isUSE_PAYLOAD()){
            //control.TurretArm().ReturnToHomeAndIntake(0.02,intakeSpeed);
            control.TurretArm().ReturnToHomeAndIntakeWithSensor();
        }
    }

    @Override
    public void YPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber && control.isUSE_PAYLOAD()){
            //control.navigation.PlaceFreightLinear();
            control.navigation.StartGoToPlace();
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
    }

    @Override
    public void XReleased(double controllerNumber) {
    }

    @Override
    public void YReleased(double controllerNumber) {

    }

    @Override
    public void LBPressed(double controllerNumber) {
        if(controllerNumber == payloadControllerNumber && control.isUSE_PAYLOAD()){
            control.TurretArm().CycleIntakeState(intakeSpeed);
        }
    }

    @Override
    public void RBPressed(double controllerNumber) {
        //toggle between duck spinner states
        if(controllerNumber == payloadControllerNumber && control.isUSE_PAYLOAD()){
            control.navigation.SpinDucks(0.5,1);
        }
        else if(controllerNumber == 2 && control.isUSE_PAYLOAD()) control.navigation.StartSpinDucks(10);
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
            control.TurretArm().StopAutoLeveller();
            control.Arm().SetPowerClamped(armSpeed);
        }
        if(controllerNumber == 2 && control.isUSE_PAYLOAD()){
            control.TurretArm().StopAutoLeveller();
            control.Arm().SetPowerRaw(armSpeed);
        }
    }

    @Override
    public void RTHeld(double controllerNumber) {
        if(controllerNumber == 1 && control.isUSE_PAYLOAD()){
            control.TurretArm().StopAutoLeveller();
            control.Arm().SetPowerClamped(-armSpeed);
        }
        if(controllerNumber == 2 && control.isUSE_PAYLOAD()){
            control.TurretArm().StopAutoLeveller();
            control.Arm().SetPowerRaw(-armSpeed);
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
        if(control.isUSE_PAYLOAD()){
            control.Arm().SetPowerRaw(0);
        }
    }

    @Override
    public void RTReleased(double controllerNumber) {
        if(control.isUSE_PAYLOAD()){
            control.Arm().SetPowerRaw(0);
        }
    }

    @Override
    public void DUpPressed(double controllerNumber) {

        control.TurretArm().AutoIntakeTierUp();
        control.TurretArm().GoToAutoTier();
    }

    @Override
    public void DDownPressed(double controllerNumber) {

        control.TurretArm().AutoIntakeTierDown();
        control.TurretArm().GoToAutoTier();
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
        if(controllerNumber == 1) {
            control.navigation.ToggleAllianceSide();
            if(control.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE) control.SetInputOffset(90); //90 is blue, -90 is red
            else if(control.navigation.side == FreightFrenzyNavigation.AllianceSide.RED) control.SetInputOffset(-90); //90 is blue, -90 is red
        }
        //if(controllerNumber == 2 && control.isUSE_PAYLOAD()) control.TurretArm().StartResetArm();
    }

    @Override
    public void RJSPressed(double controllerNumber) {
        if(controllerNumber == 1) {
            control.ResetGyro();
        }
        if(controllerNumber == 2 && control.isUSE_PAYLOAD()) control.Arm().ResetToZero();
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
