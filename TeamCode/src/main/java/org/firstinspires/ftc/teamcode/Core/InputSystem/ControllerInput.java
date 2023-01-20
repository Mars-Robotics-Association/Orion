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
    public enum Button {
        X((g) -> {return g.x;}),
        Y((g) -> {return g.y;}),
        A((g) -> {return g.a;}),
        B((g) -> {return g.b;}),
        RT((g) -> {return g.right_trigger > TriggerThreshold;}),
        LT((g) -> {return g.left_trigger > TriggerThreshold;}),
        RB((g) -> {return g.right_bumper;}),
        LB((g) -> {return g.left_bumper;}),
        DUP((g) -> {return g.dpad_up;}),
        DDOWN((g) -> {return g.dpad_down;}),
        DLEFT((g) -> {return g.dpad_left;}),
        DRIGHT((g) -> {return g.dpad_right;}),
        LJS((g) -> {return g.left_stick_button;}),
        RJS((g) -> {return g.right_stick_button;}),
        GUIDE((g) -> {return g.guide;});

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

    public void Loop(){

        //DETECT EVENTS

        //Pressed
        for(Button button : Button.values()){
            if(button.checkButton.call(gamepad) && !button.isDown)
                Pressed(button);
        }

        //Held
        for(Button button : Button.values()){
            if(button.isDown)
                Held(button);
        }

        //Released
        for(Button button : Button.values()){
            if(!button.checkButton.call(gamepad) && button.isDown)
                Released(button);
        }
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
