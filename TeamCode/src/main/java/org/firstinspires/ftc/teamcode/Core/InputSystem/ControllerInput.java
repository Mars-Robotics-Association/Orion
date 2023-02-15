package org.firstinspires.ftc.teamcode.Core.InputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

public class ControllerInput 
{
    Gamepad gamepad;
    int id;

    //A list of all the listeners
    private List<ControllerInputListener> listeners = new ArrayList<ControllerInputListener>();

    //Add a new listener
    public void addListener(ControllerInputListener toAdd) {
        listeners.add(toAdd);
    }
    
    public ControllerInput(Gamepad setGamepad, int setID){
        gamepad = setGamepad;
        id = setID;
    }

    //ENUMS
    public enum Button {X,Y,A,B,RT,LT,RB,LB,DUP,DDOWN,DLEFT,DRIGHT,LJS,RJS,GUIDE,BACK,START}

    //VARIABLES
    //misc.
    private double TriggerThreshold = 0.1;

    //buttons
    private boolean ADown = false;
    private boolean BDown = false;
    private boolean XDown = false;
    private boolean YDown = false;
    private boolean LBDown = false;
    private boolean RBDown = false;
    private boolean LTDown = false;
    private boolean RTDown = false;
    private boolean DUpDown = false;
    private boolean DDownDown = false;
    private boolean DLeftDown = false;
    private boolean DRightDown = false;
    private boolean RJSDown = false;
    private boolean LJSDown = false;
    private boolean guideDown = false;
    private boolean backDown = false;
    private boolean startDown = false;

    //GETTERS
    public Gamepad getGamepad(){return gamepad;}
    public double getLJSX()
    {
        return gamepad.left_stick_x;
    }
    public double getLJSY()
    {
        return gamepad.left_stick_y;
    }
    public double getRJSX()
    {
        return gamepad.right_stick_x;
    }
    public double getRJSY()
    {
        return gamepad.right_stick_y;
    }
    public boolean getLT(){return LTDown;}
    public boolean getRT(){return RTDown;}
    public boolean getLB(){return LBDown;}
    public boolean getRB(){return RBDown;}
    public boolean getGuide(){return guideDown;}
    public boolean getBack(){return backDown;}
    public boolean getStart(){return startDown;}

    //INTERNAL
    void pressed(Button button){
        for(ControllerInputListener listener : listeners) listener.ButtonPressed(id,button);
    }
    void held(Button button){
        for(ControllerInputListener listener : listeners) listener.ButtonHeld(id,button);
    }
    void released(Button button){
        for(ControllerInputListener listener : listeners) listener.ButtonReleased(id,button);
    }

    public void loop(){
        //DETECT EVENTS
        //Pressed
        if(gamepad.a == true && ADown == false) pressed(Button.A);
        if(gamepad.b == true && BDown == false) pressed(Button.B);
        if(gamepad.x == true && XDown == false) pressed(Button.X);
        if(gamepad.y == true && YDown == false) pressed(Button.Y);
        if(gamepad.left_bumper == true && LBDown == false) pressed(Button.LB);
        if(gamepad.right_bumper == true && RBDown == false) pressed(Button.RB);
        if(gamepad.left_trigger > TriggerThreshold && LTDown == false) pressed(Button.LT);
        if(gamepad.right_trigger > TriggerThreshold && RTDown == false) pressed(Button.RT);
        if(gamepad.dpad_up == true && DUpDown == false) pressed(Button.DUP);
        if(gamepad.dpad_down == true && DDownDown == false) pressed(Button.DDOWN);
        if(gamepad.dpad_left == true && DLeftDown == false) pressed(Button.DLEFT);
        if(gamepad.dpad_right == true && DRightDown == false) pressed(Button.DRIGHT);
        if(gamepad.left_stick_button == true && LJSDown == false) pressed(Button.LJS);
        if(gamepad.right_stick_button == true && RJSDown == false) pressed(Button.RJS);
        if(gamepad.guide == true && guideDown == false) pressed(Button.GUIDE);
        if(gamepad.back == true && backDown == false) pressed(Button.BACK);
        if(gamepad.start == true && startDown == false) pressed(Button.START);

        //Held
        if(gamepad.a == true) held(Button.A);
        if(gamepad.b == true) held(Button.B);
        if(gamepad.x == true) held(Button.X);
        if(gamepad.y == true) held(Button.Y);
        if(gamepad.left_bumper == true) held(Button.LB);
        if(gamepad.right_bumper == true) held(Button.RB);
        if(gamepad.left_trigger > TriggerThreshold) held(Button.LT);
        if(gamepad.right_trigger > TriggerThreshold) held(Button.RT);
        if(gamepad.dpad_up == true) held(Button.DUP);
        if(gamepad.dpad_down == true) held(Button.DDOWN);
        if(gamepad.dpad_left == true) held(Button.DLEFT);
        if(gamepad.dpad_right == true) held(Button.DRIGHT);
        if(gamepad.left_stick_button == true) held(Button.LJS);
        if(gamepad.right_stick_button == true) held(Button.RJS);
        if(gamepad.guide == true) held(Button.GUIDE);
        if(gamepad.back == true) held(Button.BACK);
        if(gamepad.start == true) held(Button.START);

        //Released
        if(gamepad.a == false && ADown == true) released(Button.A);
        if(gamepad.b == false && BDown == true) released(Button.B);
        if(gamepad.x == false && XDown == true) released(Button.X);
        if(gamepad.y == false && YDown == true) released(Button.Y);
        if(gamepad.left_bumper == false && LBDown == true) released(Button.LB);
        if(gamepad.right_bumper == false && RBDown == true) released(Button.RB);
        if(gamepad.left_trigger <= TriggerThreshold && LTDown == true) released(Button.LT);
        if(gamepad.right_trigger <= TriggerThreshold && RTDown == true) released(Button.RT);
        if(gamepad.dpad_up == false && DUpDown == true) released(Button.DUP);
        if(gamepad.dpad_down == false && DDownDown == true) released(Button.DDOWN);
        if(gamepad.dpad_left == false && DLeftDown == true) released(Button.DLEFT);
        if(gamepad.dpad_right == false && DRightDown == true) released(Button.DRIGHT);
        if(gamepad.left_stick_button == false && LJSDown == true) released(Button.LJS);
        if(gamepad.right_stick_button == false && RJSDown == true) released(Button.RJS);
        if(gamepad.guide == false && guideDown == true) released(Button.GUIDE);
        if(gamepad.back == false && backDown == true) released(Button.BACK);
        if(gamepad.start == false && startDown == true) released(Button.START);


        //SET VARS TO CURRENT VALUES
        ADown = gamepad.a;
        BDown = gamepad.b;
        XDown = gamepad.x;
        YDown = gamepad.y;
        LBDown = gamepad.left_bumper;
        RBDown = gamepad.right_bumper;
        LTDown = gamepad.left_trigger > TriggerThreshold;
        RTDown = gamepad.right_trigger > TriggerThreshold;
        DUpDown = gamepad.dpad_up;
        DDownDown = gamepad.dpad_down;
        DLeftDown = gamepad.dpad_left;
        DRightDown = gamepad.dpad_right;
        LJSDown = gamepad.left_stick_button;
        RJSDown = gamepad.right_stick_button;
        guideDown = gamepad.guide;
        backDown = gamepad.back;
        startDown = gamepad.start;
    }

    public double calculateLJSAngle(){
        //Calculate angle of left joystick
        double Y = gamepad.left_stick_y; //X input
        double X = gamepad.left_stick_x; //Y input

        //return telemetry for debug
        /*opMode.telemetry.addData("Joystick X ", X);
        opMode.telemetry.addData("Joystick Y ", Y);*/

        double leftStickBaring = Math.atan2(Y,X); //get measurement of joystick angle
        leftStickBaring = Math.toDegrees(leftStickBaring);
        leftStickBaring -= 90;
        if(leftStickBaring < 0)//convert degrees to positive if needed
        {
            leftStickBaring = 360 + leftStickBaring;
        }
        return leftStickBaring;
    }

    public double calculateLJSMag(){
        //Calculate magnitude of the left joystick
        //Distance formula for calculating joystick power
        return Math.abs(Math.sqrt(Math.pow(getLJSX() - 0, 2) + Math.pow(getLJSY() - 0, 2)));
    }

    public double calculateRJSAngle(){
        //Calculate angle of left joystick
        double Y = -gamepad.right_stick_y; //X input
        double X = gamepad.right_stick_x; //Y input

        //return telemetry for debug
        /*opMode.telemetry.addData("Joystick X ", X);
        opMode.telemetry.addData("Joystick Y ", Y);*/

        double rightStickBaring = Math.atan2(Y,X); //get measurement of joystick angle
        rightStickBaring = Math.toDegrees(rightStickBaring);
        rightStickBaring -= 90;
        if(rightStickBaring < 0)//convert degrees to positive if needed
        {
            rightStickBaring = 360 + rightStickBaring;
        }
        return rightStickBaring;
    }

    public double calculateRJSMag(){
        //Calculate magnitude of the left joystick
        //Distance formula for calculating joystick power
        return Math.abs(Math.sqrt(Math.pow(getRJSX() - 0, 2) + Math.pow(getRJSY() - 0, 2)));
    }
}
