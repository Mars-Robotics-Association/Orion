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
    DistanceSensor intakeSensor;
    TouchSensor levelSensor;
    ControllerInput gamepad;
    BlinkinController lights;

    enum PayloadState {MANUAL, STOPPED, LOADING, PLACING}
    enum Pole {GROUND, LOW, MID, HIGH}
    enum Side {FRONT, BACK}

    //config
    public static double armBaseSpeed = -1;
    public static double liftBaseSpeed = -1;
    public static double gripperOpenPos = 1;
    public static double gripperClosedPos = 0;
    public static double liftClearDistance = 3;
    public static double gripperTriggerDistance = 4.5;
    public static double defaultCooldown = 0.4;

    //Lift height, arm rotation
    public double[] pickupPose = {0,0};
    //ground, low, mid, high
    public double[][] placeFront = {{1,30},{4,120},{7,140},{10,160}};
    public double[][] placeBack = {{1,300},{5,270},{8,240},{10,200}};


    //STATES
    PayloadState payloadState = PayloadState.MANUAL;
    Pole targetPole = Pole.MID;
    Side targetSide = Side.FRONT;
    public void setPayloadState(PayloadState state){
        payloadState = state;
        switch (payloadState){
            case MANUAL: lights.green(); break;
            case STOPPED: lights.red(); break;
            case LOADING: lights.yellow(); break;
            case PLACING: lights.blue(); break;

        }
        lights.setCooldown(1);
    }
    public void setTargetPole(Pole pole){targetPole = pole;}
    public void setSide(Side side){targetSide=side;}

    //Keep track of stuff variables
    double lastLoopTime = 0;
    boolean liftReset = false;
    boolean hasCone = false;
    double gripperCooldown = defaultCooldown;
    double liftCooldown = defaultCooldown;
    boolean gripperOpen = false;
    boolean coneArrivedForPlacing = false;
    double currentLiftHeight = 0;


    public CuriosityPayload(OpMode setOpMode, ControllerInput setGamepad, EncoderActuator setLift,
                            EncoderActuator setArm, Servo setGripper, DistanceSensor setIntakeSensor, TouchSensor setLevelSensor, BlinkinController setLights){
        opMode = setOpMode;
        lift = setLift;
        arm = setArm;
        gripper = setGripper;
        intakeSensor = setIntakeSensor;
        gamepad = setGamepad;
        levelSensor = setLevelSensor;
        lights = setLights;
    }

    public void update(double liftInput, double armInput){
        opMode.telemetry.addData("TARGET POLE", targetPole);
        opMode.telemetry.addData("TARGET SIDE", targetSide);
        opMode.telemetry.addData("TARGET POSE", getPolePose(targetPole,targetSide)[0] + ", " + getPolePose(targetPole,targetSide)[1]);

        //manage payload states
        switch (payloadState) {
            case MANUAL: manage_raw_control(liftInput,armInput); break;
            case STOPPED: stop(); break;
            case LOADING: manageLoading(liftInput,armInput); break;
            case PLACING: managePlacing(getPolePose(targetPole,targetSide),liftInput,armInput); break;
        }
        lastLoopTime = opMode.getRuntime();

    }

    public double[] getPolePose(Pole pole, Side side){
        if(side == Side.FRONT) {
            if (pole == Pole.GROUND) return placeFront[0];
            if (pole == Pole.LOW) return placeFront[1];
            else if (pole == Pole.MID) return placeFront[2];
            else return placeFront[3];
        }
        else{
            if (pole == Pole.GROUND) return placeBack[0];
            if (pole == Pole.LOW) return placeBack[1];
            else if (pole == Pole.MID) return placeBack[2];
            else return placeBack[3];
        }
    }

    void manage_raw_control(double liftInput, double armInput){
        hasCone = true;
        coneArrivedForPlacing = false;
        arm.setPowerRaw(armInput);

        //lock the lift's position if its stopped
        /*if(liftInput == 0){
            lift.goToPosition(currentLiftHeight);
        }*/
        //if the lift is at the bottom and needs to be reset
        if(levelSensor.isPressed()&&liftInput>0) {
            liftReset=true;
            lift.resetToZero();
            lift.setPowerClamped(0);
            opMode.telemetry.addLine("Cannot Move Down Now");
            currentLiftHeight = 0;
        }
        //run the arm normally
        else{
            liftReset = false;
            lift.motors.runWithEncodersMode();
            lift.setPowerClamped(liftInput);
            currentLiftHeight = lift.getPosition();
        }
    }

    public void stop(){
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
            liftReset = true;
            lift.goToPosition(pickupPose[0]); //go to load height
            return false;
        }
        else {
            liftReset = false;
            lift.setPowerRaw(-liftBaseSpeed*.5); //goes down quickly to start
        }
        return true;
    }

    void manageLoading(double liftInput, double armInput){
        //resets arm to loading position and opens gripper
        gripper.setPosition(gripperOpenPos);
        double intakeDistance = intakeSensor.getDistance(DistanceUnit.CM);

        //telemetry
        opMode.telemetry.addLine("");
        opMode.telemetry.addData("Intake sensor distance", intakeDistance+" CM");

//        //returns if arm is not reset and there is a cone in the gripper
//        if(!liftReset && intakeDistance<=gripperTriggerDistance){
//            opMode.telemetry.addLine("Please remove cone from gripper");
//            return;
//        }

        //reset the lift's position
        if(!liftReset || Math.abs(arm.getPosition()-pickupPose[1])>0){
            if(!liftReset && autoLevel()) //reset lift
            if(!liftReset) arm.goToPosition(pickupPose[1]); //reset arm
            return;
        }
        liftReset = true;

        //allows for user tweaking
        lift.setPowerClamped(liftInput);
        arm.setPowerClamped(armInput);
        opMode.telemetry.addLine("User tweaking enabled");

        //when distance sensor detects freight, closes gripper and switches to storage state
        if(intakeDistance<=gripperTriggerDistance){//close the gripper
            opMode.telemetry.addLine("Closing gripper");
            gripper.setPosition(gripperClosedPos);
            if(gripperCooldown > 0) gripperCooldown-=getDeltaTime();//stay still for a bit to let gripper close
            else{ //then when gripper is fully closed move to next state
                gripperCooldown = defaultCooldown;
                hasCone = true;
                liftReset = false;
                coneArrivedForPlacing = false;
                stop();
                setPayloadState(PayloadState.MANUAL);
            }
        }
    }

    void managePlacing(double[] polePose, double liftInput, double armInput){
        opMode.telemetry.addLine("");
        opMode.telemetry.addLine("Managing placing!");

        //moves arm to ground target height
        if(!coneArrivedForPlacing) {
            opMode.telemetry.addData("Going to target", polePose[0] +", "+polePose[1]);
            opMode.telemetry.addData("Current height", arm.getPosition());
            lift.goToPosition(polePose[0]);
            arm.goToPosition(polePose[1]);
            //waits until its gotten high enough
            if((Math.abs(arm.getPosition()-polePose[1])<0.2)&&(Math.abs(lift.getPosition()-polePose[0])<0.2)) {
                coneArrivedForPlacing = true;
            }
            return;
        }

        //allows for user tweaking if the arm should move
        if(hasCone){
            opMode.telemetry.addLine("User tweaking enabled");
            arm.motors.runWithEncodersMode();
            arm.setPowerClamped(armInput);
            lift.motors.runWithEncodersMode();
            lift.setPowerClamped(liftInput);
        }

        //waits for user input to drop
        if(hasCone && gripper.getPosition() != gripperOpenPos) return;
        //it is dropping cone
        hasCone = false;
        opMode.telemetry.addLine("Dropping cone");

        //drops the arm down a bit
        if(coneArrivedForPlacing) {
            lift.goToPosition(lift.getPosition()-liftClearDistance); //move the arm up to avoid hitting
            coneArrivedForPlacing = false;
        }
        if(liftCooldown > 0) {
            opMode.telemetry.addLine("Waiting for lift to drop");
            liftCooldown -=getDeltaTime(); //stay still for a bit to let arm go down
            return;
        }
        else{ //then when arm is up a little bit we can move on
            liftCooldown = defaultCooldown;
        }

        //opens the gripper and waits for it to finish
        gripper.setPosition(gripperOpenPos);

        //stay still for a bit to let gripper open
        if(gripperCooldown > 0){
            opMode.telemetry.addLine("Waiting to drop cone");
            gripperCooldown-=getDeltaTime();
            return;
        }
        else if(liftCooldown == defaultCooldown){ //then when gripper is fully open move to next state
            gripperCooldown = defaultCooldown;
            lift.goToPosition(lift.getPosition()+liftClearDistance); //move the arm up to avoid hitting

        }

        //moves the lift back up
        if(liftCooldown > 0) {
            opMode.telemetry.addLine("Waiting for lift to clear top");
            liftCooldown -=getDeltaTime(); //stay still for a bit to let arm go up
            return;
        }
        else{ //then when arm is up a little bit we can move on
            liftCooldown = defaultCooldown;
            setPayloadState(PayloadState.MANUAL);
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
