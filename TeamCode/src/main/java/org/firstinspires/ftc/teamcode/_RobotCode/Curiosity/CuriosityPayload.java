package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.InputAxis;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;

public class CuriosityPayload
{
    OpMode opMode;
    EncoderActuator arm;
    Servo gripper;
    DistanceSensor gripperSensor;
    DistanceSensor armLevelSensor;
    ControllerInput gamepad;
    InputAxis armControlAxis;
    InputAxis gripperControlAxis;

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
    double gripperOpenPos = 0;
    double gripperClosedPos = 0.5;
    double gripperTriggerDistance = 0.5;
    double defaultCooldown = 0.4;

    //variables
    PayloadState payloadState = PayloadState.RAW_CONTROL;
    Pole targetPole = Pole.MID;
    public void setPayloadState(PayloadState state){payloadState = state;}
    public void setTargetPole(Pole pole){targetPole = pole;}

    //STATES
    double lastLoopTime = 0;
    boolean armReset = false;
    boolean hasCone = false;
    double gripperCooldown = defaultCooldown;
    double armCooldown = defaultCooldown;


    public CuriosityPayload(OpMode setOpMode, ControllerInput setGamepad, InputAxis setArmControlAxis, InputAxis setGripperControlAxis,
                            EncoderActuator setArm, Servo setGripper, DistanceSensor setGripperSensor, DistanceSensor setArmLevelSensor){
        opMode = setOpMode;
        arm = setArm;
        gripper = setGripper;
        gripperSensor = setGripperSensor;
        armLevelSensor = setArmLevelSensor;
        gamepad = setGamepad;
        armControlAxis = setArmControlAxis;
        gripperControlAxis = setGripperControlAxis;
    }

    public void update(){
        switch (payloadState) {
            case RAW_CONTROL: manage_raw_control(); break;
            case STOPPED: stop(); break;
            case LOADING: manageLoading(); break;
            case STORAGE: manageStorage(); break;
            case PLACING: manageTarget(getPoleHeight(targetPole)); break;
        }
        lastLoopTime = opMode.getRuntime();
    }

    double getPoleHeight(Pole pole){
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

    void manage_raw_control(){
        arm.setPowerClamped(armControlAxis.getValue());
        double gripperTargetPos = (gripperControlAxis.getValue()+1)/2;
        if(gripperTargetPos == 1 && gripper.getPosition() == gripperOpenPos) gripper.setPosition(gripperClosedPos);
        else if (gripperTargetPos == 0 && gripper.getPosition() == gripperClosedPos) gripper.setPosition(gripperOpenPos);
    }

    public void stop(){
        //stops all motors
        arm.setPowerRaw(0);
    }

    void manageLoading(){
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
        arm.setPowerClamped(armControlAxis.getValue());

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

    void manageStorage(){
        //moves arm to neutral state for driving around
        arm.goToPosition(storageHeight);
        gripper.setPosition(gripperClosedPos);
    }

    void manageTarget(double poleHeight){
        //moves arm to ground target height
        arm.goToPosition(poleHeight);

        //allows for user tweaking if the arm should move
        if(hasCone) arm.setPowerClamped(armControlAxis.getValue());

        //waits for user input to drop
        if(gripperControlAxis.getValue() != 1) return;

        //opens the gripper and waits for it to finish
        gripper.setPosition(gripperOpenPos);
        if(gripperCooldown > 0){
            gripperCooldown-=getDeltaTime(); //stay still for a bit to let gripper open
            return;
        }
        else{ //then when gripper is fully open move to next state
            gripperCooldown = defaultCooldown;
            payloadState = PayloadState.RAW_CONTROL;
            hasCone = false;
            arm.goToPosition(arm.getPosition()+armClearHeight); //move the arm up to avoid hitting
        }

        //wait for the arm to clear top
        if(armCooldown > 0) armCooldown-=getDeltaTime(); //stay still for a bit to let arm go up
        else{ //then when arm is up a little bit we can move on
            armCooldown = defaultCooldown;
            payloadState = PayloadState.RAW_CONTROL;
        }
    }

    double getDeltaTime(){
        return opMode.getRuntime() - lastLoopTime;
    }
}