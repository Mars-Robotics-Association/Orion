package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.InputAxis;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;

public class CuriosityPayload
{
    OpMode opMode;
    EncoderActuator arm;
    Servo gripper;
    DistanceSensor gripperSensor;
    DistanceSensor armLevelSensor;
    ControllerInput gamepad;
    BlinkinController lights;

    enum PayloadState {RAW_CONTROL, STOPPED, LOADING, STORAGE, PLACING}
    enum Pole {GROUND, LOW, MID, HIGH}

    //config
    double loadHeight = 5;
    double storageHeight = 15;
    double armClearHeight = 5;
    double armLevelDistance = 2;
    double armSlowDistance = 5;
    double armBaseSpeed = 1;
    double armSlowSpeed = 0.5;
    double gripperOpenPos = 1;
    double gripperClosedPos = 0;
    double gripperTriggerDistance = 0.5;
    double defaultCooldown = 0.4;

    //variables
    PayloadState payloadState = PayloadState.RAW_CONTROL;
    Pole targetPole = Pole.MID;
    public void setPayloadState(PayloadState state){
        payloadState = state;
        switch (payloadState){
            case RAW_CONTROL: lights.green(); break;
            case STOPPED: lights.red(); break;
            case LOADING: lights.yellow(); break;
            case STORAGE: lights.blue(); break;
            case PLACING: lights.purple(); break;

        }
        lights.setCooldown(1);
    }
    public void setTargetPole(Pole pole){targetPole = pole;}

    //STATES
    double lastLoopTime = 0;
    boolean armReset = false;
    boolean hasCone = false;
    double gripperCooldown = defaultCooldown;
    double armCooldown = defaultCooldown;
    boolean gripperOpen = false;


    public CuriosityPayload(OpMode setOpMode, ControllerInput setGamepad,
                            EncoderActuator setArm, Servo setGripper, DistanceSensor setGripperSensor, DistanceSensor setArmLevelSensor){
        opMode = setOpMode;
        arm = setArm;
        gripper = setGripper;
        gripperSensor = setGripperSensor;
        armLevelSensor = setArmLevelSensor;
        gamepad = setGamepad;
    }

    public void update(double armInput){
        //clamps inputs to correct range
        armInput*=armBaseSpeed;
        armInput = Math.max(-armBaseSpeed, Math.min(armBaseSpeed, armInput));
        //manage payload states
        switch (payloadState) {
            case RAW_CONTROL: manage_raw_control(armInput); break;
            case STOPPED: stop(); break;
            case LOADING: manageLoading(armInput); break;
            case STORAGE: manageStorage(armInput); break;
            case PLACING: manageTarget(getPoleHeight(targetPole), armInput); break;
        }
        lastLoopTime = opMode.getRuntime();
    }

    public static double getPoleHeight(Pole pole){
        if (pole == Pole.LOW) return 15;
        else if (pole == Pole.MID) return 24;
        else return 36;
    }

    void moveArmByAmount(double amount){
        //adds amount to current arm target position
        double armTargetPosition = arm.getPosition()+amount;
        //clamp value
        if(armTargetPosition>arm.maxRots) armTargetPosition = arm.maxRots;
        if(armTargetPosition< arm.minRots) armTargetPosition = arm.minRots;
        //set power and go to position
        arm.setPowerRaw(armBaseSpeed);
        arm.goToPosition(armTargetPosition);
    }

    void manage_raw_control(double armInput){
        arm.setPowerRaw(armInput);
    }

    public void stop(){
        //stops all motors
        arm.setPowerRaw(0);
    }

    void manageLoading(double armInput){
        //resets arm to loading position and opens gripper
        gripper.setPosition(gripperOpenPos);
        double armDistanceFromGround = armLevelSensor.getDistance(DistanceUnit.INCH);
        if(!armReset) { //reset the arm's position
            if(armDistanceFromGround <= armLevelDistance){ //if at bottom, reset arm's position
                arm.resetToZero();
                armReset = true;
                arm.goToPosition(loadHeight); //go to load height
                arm.setPowerRaw(armBaseSpeed);
            }
            else if (armDistanceFromGround <= armSlowDistance) arm.setPowerRaw(-armSlowSpeed); //slows down if close to bottom
            else if (armDistanceFromGround > armSlowDistance) arm.setPowerRaw(-armBaseSpeed); //goes down quickly to start
            return; //don't do anything else if resetting the arm's position
        }

        //allows for user tweaking
        arm.setPowerClamped(armInput);

        //when distance sensor detects freight, closes gripper and switches to storage state
        if(gripperSensor.getDistance(DistanceUnit.INCH)<=gripperTriggerDistance){//close the gripper
            gripper.setPosition(gripperClosedPos);
            if(gripperCooldown > 0) gripperCooldown-=getDeltaTime();//stay still for a bit to let gripper close
            else{ //then when gripper is fully closed move to next state
                gripperCooldown = defaultCooldown;
                hasCone = true;
                payloadState = PayloadState.STORAGE;
            }
        }
    }

    void manageStorage(double armInput){
        //moves arm to neutral state for driving around
        arm.goToPosition(storageHeight);
        gripper.setPosition(gripperClosedPos);
    }

    void manageTarget(double poleHeight, double armInput){
        //moves arm to ground target height
        arm.goToPosition(poleHeight);

        //allows for user tweaking if the arm should move
        if(hasCone){
            arm.motors.runWithEncodersMode();
            arm.setPowerClamped(armInput);
        }

        //waits for user input to drop
        if(hasCone && gripper.getPosition() != gripperOpenPos) return;

        //opens the gripper and waits for it to finish
        gripper.setPosition(gripperOpenPos);
        //it has dropped cone
        hasCone = false;

        if(gripperCooldown > 0){
            gripperCooldown-=getDeltaTime(); //stay still for a bit to let gripper open
            return;
        }
        else{ //then when gripper is fully open move to next state
            gripperCooldown = defaultCooldown;
            payloadState = PayloadState.RAW_CONTROL;
            arm.goToPosition(arm.getPosition()+armClearHeight); //move the arm up to avoid hitting
        }

        //wait for the arm to clear top
        if(armCooldown > 0) {
            armCooldown-=getDeltaTime(); //stay still for a bit to let arm go up
            return;
        }
        else{ //then when arm is up a little bit we can move on
            armCooldown = defaultCooldown;
            payloadState = PayloadState.RAW_CONTROL;
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
}
