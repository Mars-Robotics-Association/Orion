package org.firstinspires.ftc.teamcode.Core.InputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;

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

    interface CheckButton{
        boolean call(Gamepad gamepad);
    }

    //ENUMS
    public enum Button {X,Y,A,B,RT,LT,RB,LB,DUP,DDOWN,DLEFT,DRIGHT,LJS,RJS,GUIDE}

        private final boolean isDown = false;

        private final CheckButton checkButton;

        Button(CheckButton checkButton){
            this.checkButton = checkButton;
        }
    }

    private final Button[] allButtons = Button.values();

    //VARIABLES
    //misc.
    private static double TriggerThreshold = 0.1;

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

    //GETTERS
    public double GetLJSX()
    {
        return gamepad.left_stick_x;
    }
    public double GetLJSY()
    {
        return gamepad.left_stick_y;
    }
    public double GetRJSX()
    {
        return gamepad.right_stick_x;
    }
    public double GetRJSY()
    {
        return gamepad.right_stick_y;
    }
    public boolean getLT(){return LTDown;}
    public boolean getRT(){return RTDown;}
    public boolean getLB(){return LBDown;}
    public boolean getRB(){return RBDown;}
    public boolean getGuide(){return guideDown;}

    //INTERNAL
    void Pressed(Button button){
        for(ControllerInputListener listener : listeners) listener.ButtonPressed(id,button);
    }
    void Held(Button button){
        for(ControllerInputListener listener : listeners) listener.ButtonHeld(id,button);
    }
    void Released(Button button){
        for(ControllerInputListener listener : listeners) listener.ButtonReleased(id,button);
    }

    public void Loop(){

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
    }

    public double CalculateLJSAngle(){
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

    public double CalculateLJSMag(){
        //Calculate magnitude of the left joystick
        //Distance formula for calculating joystick power
        return Math.abs(Math.sqrt(Math.pow(GetLJSX() - 0, 2) + Math.pow(GetLJSY() - 0, 2)));
    }

    public enum Stick {
        LEFT_STICK,
        RIGHT_STICK
    }

    public double getStickAngle(Stick stick){
        double X = gamepad.left_stick_x;
        double Y = gamepad.left_stick_y;

        switch(stick){
            case LEFT_STICK:
                X = gamepad.left_stick_x;
                Y = gamepad.left_stick_y;
                break;
            case RIGHT_STICK:
                X = gamepad.right_stick_x;
                Y = gamepad.right_stick_y;
        }

        double baring = Math.atan2(Y,X); //get measurement of joystick angle
        baring = Math.toDegrees(baring);
        baring -= 90;
        if(baring < 0)baring += 360;//convert degrees to positive if needed
        return baring;
    }

    public double getStickMag(Stick stick){
        double X = gamepad.left_stick_x;
        double Y = gamepad.left_stick_y;

        switch(stick){
            case LEFT_STICK:
                X = gamepad.left_stick_x;
                Y = gamepad.left_stick_y;
                break;
            case RIGHT_STICK:
                X = gamepad.right_stick_x;
                Y = gamepad.right_stick_y;
        }

        double baring = Math.atan2(Y,X); //get measurement of joystick angle
        baring = Math.toDegrees(baring);
        baring -= 90;
        if(baring < 0)baring += 360;//convert degrees to positive if needed
        return baring;
    }
}
