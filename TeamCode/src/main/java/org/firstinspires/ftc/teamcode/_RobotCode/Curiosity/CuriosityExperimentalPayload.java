package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;

@Config
public class CuriosityExperimentalPayload
{
    OpMode opMode;
    EncoderActuator arm;
    public EncoderActuator getArm(){return arm;}
    EncoderActuator lift;
    public EncoderActuator getLift(){return lift;}
    Servo gripper;
    DistanceSensor gripperSensor;
    DistanceSensor armLevelSensor;
    ControllerInput gamepad;
    BlinkinController lights;

    enum PayloadState {MANUAL, STOPPED, LOADING, PLACING}
    enum Pole {GROUND, LOW, MID, HIGH}
    enum Side {FRONT, BACK}

    //config
    public static double loadHeight = 0;
    public static double armClearHeight = 3;
    public static double armLevelDistance = 5;
    public static double armSlowDistance = 10;
    public static double armBaseSpeed = 1;
    public static double armSlowSpeed = 0.5;
    public static double gripperOpenPos = 1;
    public static double gripperClosedPos = 0;
    public static double gripperTriggerDistance = 4.5;
    public static double defaultCooldown = 0.4;

    public static double groundJunction = 5;
    public static double lowJunction = 20;
    public static double midJunction = 26;
    public static double highJunction = 39;

    //variables
    PayloadState payloadState = PayloadState.MANUAL;
    Pole targetPole = Pole.MID;
    public void setPayloadState(PayloadState state){
        payloadState = state;
//        switch (payloadState){
//            case RAW_CONTROL: lights.green(); break;
//            case STOPPED: lights.red(); break;
//            case LOADING: lights.yellow(); break;
//            case STORAGE: lights.blue(); break;
//            case PLACING: lights.purple(); break;
//
//        }
//        lights.setCooldown(1);
    }
    public void setTargetPole(Pole pole){targetPole = pole;}

    //STATES
    double lastLoopTime = 0;
    boolean armReset = false;
    boolean hasCone = false;
    double gripperCooldown = defaultCooldown;
    double armCooldown = defaultCooldown;
    boolean gripperOpen = false;
    boolean armArrivedAtHeight = false;


    public CuriosityExperimentalPayload(OpMode setOpMode, ControllerInput setGamepad,
                            EncoderActuator setArm, Servo setGripper, DistanceSensor setGripperSensor, DistanceSensor setArmLevelSensor){
        opMode = setOpMode;
        arm = setArm;
        gripper = setGripper;
        gripperSensor = setGripperSensor;
        armLevelSensor = setArmLevelSensor;
        gamepad = setGamepad;
    }

    public void update(double armInput){
        opMode.telemetry.addData("CURRENT TARGET POLE", targetPole);
        //clamps inputs to correct range
        armInput*=armBaseSpeed;
        armInput = Math.max(-armBaseSpeed, Math.min(armBaseSpeed, armInput));
        //manage payload states
        switch (payloadState) {
            case MANUAL: manage_raw_control(armInput); break;
            case STOPPED: stop(); break;
            case LOADING: manageLoading(armInput); break;
            case PLACING: manageTarget(getPoleHeight(targetPole),armInput); break;
        }
        lastLoopTime = opMode.getRuntime();
    }

    public static double getPoleHeight(Pole pole){
        if(pole == Pole.GROUND) return groundJunction;
        if (pole == Pole.LOW) return lowJunction;
        else if (pole == Pole.MID) return midJunction;
        else return highJunction;
    }

    void manage_raw_control(double armInput){
        hasCone = true;
        armReset = false;
        armArrivedAtHeight = false;
        arm.setPowerClamped(armInput);
    }

    public void stop(){
        //stops all motors
        arm.motors.runWithEncodersMode();
        arm.setPowerRaw(0);
    }

    //returns false when done
    boolean autoLevel(){
        double armDistanceFromGround = armLevelSensor.getDistance(DistanceUnit.CM);
        opMode.telemetry.addLine("Resetting arm");
        if(armDistanceFromGround <= armLevelDistance){ //if at bottom, reset arm's position
            arm.resetToZero();
            armReset = true;
            arm.goToPosition(loadHeight); //go to load height
            return false;
        }
        else if (armDistanceFromGround <= armSlowDistance) {
            opMode.telemetry.addLine("Slowing down arm");
            arm.setPowerRaw(-armSlowSpeed); //slows down if close to bottom
        }
        else arm.setPowerRaw(-armBaseSpeed); //goes down quickly to start
        return true;
    }

    void manageLoading(double armInput){
        //resets arm to loading position and opens gripper
        gripper.setPosition(gripperOpenPos);
        double armDistanceFromGround = armLevelSensor.getDistance(DistanceUnit.CM);
        double gripperDistance = gripperSensor.getDistance(DistanceUnit.CM);

        //telemetry
        opMode.telemetry.addLine("");
        opMode.telemetry.addData("Arm dist from ground", armDistanceFromGround+" CM");
        opMode.telemetry.addData("Gripper sensor distance", gripperDistance+" CM");

        //returns if arm is not reset and there is a cone in the gripper
        if(!armReset && gripperDistance<=gripperTriggerDistance){
            opMode.telemetry.addLine("Please remove cone from gripper");
            return;
        }

        //reset the arm's position
        if(!armReset) {
            opMode.telemetry.addLine("Resetting arm");
            if(armDistanceFromGround <= armLevelDistance){ //if at bottom, reset arm's position
                arm.resetToZero();
                armReset = true;
                arm.goToPosition(loadHeight); //go to load height
            }
            else if (armDistanceFromGround <= armSlowDistance) {
                opMode.telemetry.addLine("Slowing down arm");
                arm.setPowerRaw(-armSlowSpeed); //slows down if close to bottom
            }
            else arm.setPowerRaw(-armBaseSpeed); //goes down quickly to start
            return; //don't do anything else if resetting the arm's position
        }

        //allows for user tweaking
        arm.setPowerClamped(armInput);
        opMode.telemetry.addLine("User tweaking enabled");

        //when distance sensor detects freight, closes gripper and switches to storage state
        if(gripperDistance<=gripperTriggerDistance){//close the gripper
            opMode.telemetry.addLine("Closing gripper");
            gripper.setPosition(gripperClosedPos);
            if(gripperCooldown > 0) gripperCooldown-=getDeltaTime();//stay still for a bit to let gripper close
            else{ //then when gripper is fully closed move to next state
                gripperCooldown = defaultCooldown;
                hasCone = true;
                armReset = false;
                armArrivedAtHeight = false;
                stop();
                setPayloadState(PayloadState.MANUAL);
            }
        }
    }

    void manageTarget(double poleHeight, double armInput){
        opMode.telemetry.addLine("");
        opMode.telemetry.addLine("Managing placing!");

        //moves arm to ground target height
        if(!armArrivedAtHeight) {
            opMode.telemetry.addData("Going to target", poleHeight);
            opMode.telemetry.addData("Current height", arm.getPosition());
            arm.goToPosition(poleHeight);
            //waits until its gotten high enough
            if(Math.abs(arm.getPosition()-poleHeight)<1) {
                armArrivedAtHeight = true;
            }
            return;
        }

        //allows for user tweaking if the arm should move
        if(hasCone){
            opMode.telemetry.addLine("User tweaking enabled");
            arm.motors.runWithEncodersMode();
            arm.setPowerClamped(armInput);
        }

        //waits for user input to drop
        if(hasCone && gripper.getPosition() != gripperOpenPos) return;

        opMode.telemetry.addLine("Dropping cone");

        //opens the gripper and waits for it to finish
        gripper.setPosition(gripperOpenPos);
        //it has dropped cone
        hasCone = false;

        if(gripperCooldown > 0){
            opMode.telemetry.addLine("Waiting to drop cone");
            gripperCooldown-=getDeltaTime(); //stay still for a bit to let gripper open
            return;
        }
        else{ //then when gripper is fully open move to next state
            gripperCooldown = defaultCooldown;
            arm.goToPosition(arm.getPosition()+armClearHeight); //move the arm up to avoid hitting
            armArrivedAtHeight = false;
            setPayloadState(PayloadState.MANUAL);
        }

//        //wait for the arm to clear top
//        if(armCooldown > 0) {
//            opMode.telemetry.addLine("Waiting for arm to clear top");
//            armCooldown-=getDeltaTime(); //stay still for a bit to let arm go up
//            return;
//        }
//        else{ //then when arm is up a little bit we can move on
//            armCooldown = defaultCooldown;
//
//        }
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
