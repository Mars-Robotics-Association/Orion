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
    public enum Button {X,Y,A,B,RT,LT,RB,LB,DUP,DDOWN,DLEFT,DRIGHT,LJS,RJS}

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
        if(gamepad.a == true && ADown == false) Pressed(Button.A);
        if(gamepad.b == true && BDown == false) Pressed(Button.B);
        if(gamepad.x == true && XDown == false) Pressed(Button.X);
        if(gamepad.y == true && YDown == false) Pressed(Button.Y);
        if(gamepad.left_bumper == true && LBDown == false) Pressed(Button.LB);
        if(gamepad.right_bumper == true && RBDown == false) Pressed(Button.RB);
        if(gamepad.left_trigger > TriggerThreshold && LTDown == false) Pressed(Button.LT);
        if(gamepad.right_trigger > TriggerThreshold && RTDown == false) Pressed(Button.RT);
        if(gamepad.dpad_up == true && DUpDown == false) Pressed(Button.DUP);
        if(gamepad.dpad_down == true && DDownDown == false) Pressed(Button.DDOWN);
        if(gamepad.dpad_left == true && DLeftDown == false) Pressed(Button.DLEFT);
        if(gamepad.dpad_right == true && DRightDown == false) Pressed(Button.DRIGHT);
        if(gamepad.left_stick_button == true && LJSDown == false) Pressed(Button.LJS);
        if(gamepad.right_stick_button == true && RJSDown == false) Pressed(Button.RJS);

        //Held
        if(gamepad.a == true) Held(Button.A);
        if(gamepad.b == true) Held(Button.B);
        if(gamepad.x == true) Held(Button.X);
        if(gamepad.y == true) Held(Button.Y);
        if(gamepad.left_bumper == true) Held(Button.LB);
        if(gamepad.right_bumper == true) Held(Button.RB);
        if(gamepad.left_trigger > TriggerThreshold) Held(Button.LT);
        if(gamepad.right_trigger > TriggerThreshold) Held(Button.RT);
        if(gamepad.dpad_up == true) Held(Button.DUP);
        if(gamepad.dpad_down == true) Held(Button.DDOWN);
        if(gamepad.dpad_left == true) Held(Button.DLEFT);
        if(gamepad.dpad_right == true) Held(Button.DRIGHT);
        if(gamepad.left_stick_button == true) Held(Button.LJS);
        if(gamepad.right_stick_button == true) Held(Button.RJS);

        //Released
        if(gamepad.a == false && ADown == true) Released(Button.A);
        if(gamepad.b == false && BDown == true) Released(Button.B);
        if(gamepad.x == false && XDown == true) Released(Button.X);
        if(gamepad.y == false && YDown == true) Released(Button.Y);
        if(gamepad.left_bumper == false && LBDown == true) Released(Button.LB);
        if(gamepad.right_bumper == false && RBDown == true) Released(Button.RB);
        if(gamepad.left_trigger <= TriggerThreshold && LTDown == true) Released(Button.LT);
        if(gamepad.right_trigger <= TriggerThreshold && RTDown == true) Released(Button.RT);
        if(gamepad.dpad_up == false && DUpDown == true) Released(Button.DUP);
        if(gamepad.dpad_down == false && DDownDown == true) Released(Button.DDOWN);
        if(gamepad.dpad_left == false && DLeftDown == true) Released(Button.DLEFT);
        if(gamepad.dpad_right == false && DRightDown == true) Released(Button.DRIGHT);
        if(gamepad.left_stick_button == false && LJSDown == true) Released(Button.LJS);
        if(gamepad.right_stick_button == false && RJSDown == true) Released(Button.RJS);


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
}
