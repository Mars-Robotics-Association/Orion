package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;

@Config
public class CuriosityPayload
{
    OpMode opMode;
    EncoderActuator arm;
    public EncoderActuator getArm(){return arm;}
    EncoderActuator lift;
    public EncoderActuator getLift(){return lift;}
    Servo gripper;
    Servo gripperLeveller;
    DistanceSensor intakeSensor;
    TouchSensor levelSensor;
    ControllerInput gamepad;
    BlinkinController lights;

    enum PayloadState {MANUAL, STOPPED, LOADING, PLACING, STORAGE}
    enum Pole {GROUND, LOW, MID, HIGH}

    //config
    public static double gripperOpenPos = 0.88;
    public static double gripperClosedPos = 1;
    public static double placeDropDistance = 3;
    public static double gripperTriggerDistance = 4;
    public static double LIFT_COOLDOWN = 0.6;
    public static double GRIPPER_COOLDOWN = 0.4;
    public static double gripperLevelPlaceStart = 5;//degrees of arm to rotate for placing
    public static double gripperLevelPlaceOffset = -40;//degrees to rotate gripper further for placing
    public static double gripperLevelCoefficient = -150;//1 is 150 degree rotation on end
    public static double gripperLevelOverallOffset = 0;

    //Lift height, arm rotation
    public double[] pickupPose = {6,0};
    //ground, low, mid, high
    public double[][] placeFront = {{2,20},{11,50},{4,170},{12,170}};


    //STATES
    PayloadState payloadState = PayloadState.MANUAL;
    Pole targetPole = Pole.MID;
    public void setPayloadState(PayloadState state){
        payloadState = state;
        switch (payloadState){
            case MANUAL: lights.green(); break;
            case STOPPED: lights.red(); break;
            case LOADING: lights.yellow(); break;
            case PLACING: lights.purple(); break;
            //case STORAGE: lights.blue(); break;

        }
    }
    public void setTargetPole(Pole pole){targetPole = pole;}

    //Keep track of stuff variables
    private enum LoadSubstate{ALIGNING, READY, DROPPING, CLOSING, RAISING}
    LoadSubstate loadSubstate = LoadSubstate.ALIGNING;
    private enum PlaceSubstate{ALIGNING, READY, LOWERING, RELEASING, RAISING}
    PlaceSubstate placeSubstate = PlaceSubstate.ALIGNING;
    double lastLoopTime = 0;
    double gripperCooldown = GRIPPER_COOLDOWN;
    double liftCooldown = LIFT_COOLDOWN;
    boolean gripperOpen = false;
    double currentLiftHeight = 0;


    public CuriosityPayload(OpMode setOpMode, ControllerInput setGamepad, EncoderActuator setLift,
                            EncoderActuator setArm, Servo setGripper, Servo setGripperLeveller, DistanceSensor setIntakeSensor, TouchSensor setLevelSensor, BlinkinController setLights){
        opMode = setOpMode;
        lift = setLift;
        arm = setArm;
        gripper = setGripper;
        gripperLeveller = setGripperLeveller;
        intakeSensor = setIntakeSensor;
        gamepad = setGamepad;
        levelSensor = setLevelSensor;
        lights = setLights;
    }

    public void update(double liftInput, double armInput){
        opMode.telemetry.addData("TARGET POLE", targetPole);
        opMode.telemetry.addData("TARGET POSE", getPolePose(targetPole)[0] + ", " + getPolePose(targetPole)[1]);

        //manage payload states
        switch (payloadState) {
            case MANUAL: manage_raw_control(liftInput,armInput); break;
            case STOPPED: stop(); break;
            case LOADING: manageLoading(liftInput,armInput); break;
            case STORAGE: manageStorage(); break;
            case PLACING: managePlacing(getPolePose(targetPole),liftInput,armInput); break;
        }

        lastLoopTime = opMode.getRuntime();
        if(payloadState!=PayloadState.STOPPED) levelGripper();
    }

    public void manageStorage(){
        gripperCooldown = LIFT_COOLDOWN;
        loadSubstate = LoadSubstate.ALIGNING;
        placeSubstate = PlaceSubstate.ALIGNING;
        lift.goToPosition(pickupPose[0]);
        arm.goToPosition(pickupPose[1]);
    }

    public void levelGripper(){
        if(arm.getPosition()>=gripperLevelPlaceStart || lift.getPosition() > 2) {
            opMode.telemetry.addData("Rotating gripper to ", (arm.getPosition()+gripperLevelPlaceOffset+gripperLevelOverallOffset));
            gripperLeveller.setPosition(1+((arm.getPosition()+gripperLevelPlaceOffset+gripperLevelOverallOffset)/gripperLevelCoefficient));
        }
        else gripperLeveller.setPosition(1+(gripperLevelOverallOffset/gripperLevelCoefficient));
    }

    public void changeGripperLevelOffset(double amount){
        gripperLevelOverallOffset += amount;
    }

    public double[] getPolePose(Pole pole){
        if (pole == Pole.GROUND) return placeFront[0];
        if (pole == Pole.LOW) return placeFront[1];
        else if (pole == Pole.MID) return placeFront[2];
        else return placeFront[3];

    }

    void manage_raw_control(double liftInput, double armInput){
        loadSubstate = LoadSubstate.ALIGNING;
        placeSubstate = PlaceSubstate.ALIGNING;
        arm.setPowerRaw(armInput);

        //if the lift is at the bottom and needs to be reset
        if(levelSensor.isPressed()&&liftInput>0) {
            lift.resetToZero();
            lift.setPowerClamped(0);
            opMode.telemetry.addLine("Cannot Move Down Now");
            currentLiftHeight = 0;
        }
        //run the arm normally
        else{
            lift.motors.runWithEncodersMode();
            lift.setPowerClamped(liftInput);
            currentLiftHeight = lift.getPosition();
        }
    }

    public void stop(){
        loadSubstate = LoadSubstate.ALIGNING;
        placeSubstate = PlaceSubstate.ALIGNING;
        //stops all motors
        arm.motors.runWithEncodersMode();
        arm.setPowerRaw(0);
        lift.motors.runWithEncodersMode();
        lift.setPowerRaw(0);
    }

    //returns false when done
    boolean autoLevel(){
        opMode.telemetry.addLine("Resetting arm");
        if(levelSensor.isPressed()){ //if at bottom, reset arm's position
            lift.resetToZero();
            //lift.goToPosition(pickupPose[0]); //go to load height
            return false;
        }
        else {
            lift.setPowerRaw(1); //goes down
        }
        return true;
    }

    void manageLoading(double liftInput, double armInput){
        //resets arm to loading position and opens gripper
        double intakeDistance = intakeSensor.getDistance(DistanceUnit.CM);

        //telemetry
        opMode.telemetry.addLine("");
        opMode.telemetry.addLine("* L O A D I N G *");
        opMode.telemetry.addData("Intake sensor distance", intakeDistance+" CM");

        switch (loadSubstate){
            case ALIGNING:
                opMode.telemetry.addData("Substate", "ALIGNING");
                toggleGripper(true);
                gripperCooldown = GRIPPER_COOLDOWN;
                //moves the lift into pickup position
                if ((Math.abs(lift.getPosition() - pickupPose[0]) > 0.2) || (Math.abs(arm.getPosition() - pickupPose[1]) > 0.2)) {
                    lift.goToPosition(pickupPose[0]);
                    arm.goToPosition(pickupPose[1]);}
                //once arrived, move on to ready state
                else loadSubstate = LoadSubstate.READY;
                break;

            case READY:
                opMode.telemetry.addData("Substate", "READY");
                //allow for user tweaking
                lift.setPowerClamped(liftInput);
                arm.setPowerClamped(armInput);
                //when distance sensor detects freight, drops arm and then closes gripper
                if(intakeDistance<=gripperTriggerDistance) loadSubstate = LoadSubstate.DROPPING;
                break;

            case DROPPING:
                opMode.telemetry.addData("Substate", "DROPPING");
                //auto level the lift and move arm in
                if (autoLevel()) arm.goToPosition(0);
                //move on once arm is level
                else loadSubstate = LoadSubstate.CLOSING;
                break;

            case CLOSING:
                opMode.telemetry.addData("Substate", "CLOSING");
                //close the gripper
                toggleGripper(false);
                //stay still for a bit to let gripper close
                if(gripperCooldown > 0) gripperCooldown -=getDeltaTime();
                // when gripper is fully closed move to next state
                else {
                    gripperCooldown = GRIPPER_COOLDOWN;
                    lights.green();
                    loadSubstate = LoadSubstate.RAISING;}
                break;

            case RAISING:
                opMode.telemetry.addData("Substate", "RAISING");
                //moves the lift into pickup position
                if ((Math.abs(lift.getPosition() - pickupPose[0]) > 0.4)) lift.goToPosition(pickupPose[0]);
                //once arrived, switch to manual mode
                else setPayloadState(PayloadState.MANUAL);
                break;
        }
    }

    void managePlacing(double[] polePose, double liftInput, double armInput){
        opMode.telemetry.addLine("");
        opMode.telemetry.addLine("* P L A C I N G *");

        switch (placeSubstate){
            case ALIGNING:
                opMode.telemetry.addData("Substate", "ALIGNING");
                toggleGripper(false);
                gripperCooldown = GRIPPER_COOLDOWN;
                //moves the lift into placing position
                if ((Math.abs(lift.getPosition() - polePose[0]) > 0.2) || (Math.abs(arm.getPosition() - polePose[1]) > 0.2)) {
                    lift.goToPosition(polePose[0]);
                    arm.goToPosition(polePose[1]);}
                //once arrived, move on to ready state
                else placeSubstate = PlaceSubstate.READY;
                break;

            case READY:
                opMode.telemetry.addData("Substate", "READY");
                //allow for user tweaking
                lift.setPowerClamped(liftInput);
                arm.setPowerClamped(armInput);
                //wait for the gripper to open, then move on
                if(gripperOpen) {
                    lights.yellow();
                    placeSubstate = PlaceSubstate.LOWERING;
                }
                liftCooldown = LIFT_COOLDOWN;
                break;

            case LOWERING:
                opMode.telemetry.addData("Substate", "LOWERING");
                //drops the lift down a bit
                if (liftCooldown > 0) {
                    lift.goToPosition(polePose[0]-placeDropDistance);
                    liftCooldown -= getDeltaTime();}
                else {
                    placeSubstate = PlaceSubstate.RELEASING;
                    liftCooldown = LIFT_COOLDOWN;}
                break;

            case RELEASING:
                opMode.telemetry.addData("Substate", "RELEASING");
                //open the gripper
                toggleGripper(true);
                //stay still for a bit to let gripper close
                if(gripperCooldown > 0) gripperCooldown -=getDeltaTime();
                // when gripper is fully open move to next state
                else {
                    gripperCooldown = GRIPPER_COOLDOWN;
                    lights.green();
                    placeSubstate = PlaceSubstate.RAISING;}
                break;

            case RAISING:
                opMode.telemetry.addData("Substate", "RAISING");
                if (liftCooldown > 0) {
                    lift.goToPosition(polePose[0]);
                    liftCooldown -= getDeltaTime();}
                else {
                    liftCooldown = LIFT_COOLDOWN;
                    setPayloadState(PayloadState.MANUAL);}
                break;
        }
    }

    double getDeltaTime(){
        return opMode.getRuntime() - lastLoopTime;
    }

    public void toggleGripper(boolean open){
        if(open) {
            gripper.setPosition(gripperOpenPos);
            gripperOpen = true;
        }
        else {
            gripper.setPosition(gripperClosedPos);
            gripperOpen = false;
        }
    }

    public void toggleGripper(){
        if(gripperOpen) toggleGripper(false);
        else toggleGripper(true);
    }

    public void goToHeight(double height){
        arm.goToPosition(height);
    }
}
