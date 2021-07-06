package org.firstinspires.ftc.teamcode.Core.Input;

import com.qualcomm.robotcore.hardware.Gamepad;


import java.util.ArrayList;
import java.util.List;

//Calls event based commands from a controller as well as providing direct access. To use, make a
//class interface to ControllerInputListener.java, create an instance of this class, call input.AddListener(this),
//then input.Init(), then there will be errors so click on red light bulb and hit "implement methods".

//REQUIRED TO RUN: None
//REQUIRED TO FUNCTION: Controller

public class ControllerInput
{
    //REFERENCES
    private Gamepad gamepad;
    private int id;
    
    //A list of all the listeners
    private List<ControllerInputListener> listeners = new ArrayList<ControllerInputListener>();

    //Add a new listener
    public void addListener(ControllerInputListener toAdd) {
        listeners.add(toAdd);
    }
    
    //Initializer
    public ControllerInput(Gamepad setGamepad, int setId){
        gamepad = setGamepad;
        id = setId;
    }

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

    ////EVENTS////
    //Pressed
    public void APressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.APressed(id);
    }
    public void BPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.BPressed(id);
    }
    public void XPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.XPressed(id);
    }
    public void YPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.YPressed(id);
    }
    public void RBPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RBPressed(id);
    }
    public void LBPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.LBPressed(id);
    }
    public void RTPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RTPressed(id);
    }
    public void LTPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.LTPressed(id);
    }
    public void DUpPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DUpPressed(id);
    }
    public void DDownPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DDownPressed(id);
    }
    public void DLeftPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DLeftPressed(id);
    }
    public void DRightPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DRightPressed(id);
    }
    public void LJSPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.LJSPressed(id);
    }
    public void RJSPressed() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RJSPressed(id);
    }

    //Held
    public void AHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.AHeld(id);
    }
    public void BHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.BHeld(id);
    }
    public void XHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.XHeld(id);
    }
    public void YHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.YHeld(id);
    }
    public void RBHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RBHeld(id);
    }
    public void LBHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.LBHeld(id);
    }
    public void RTHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RTHeld(id);
    }
    public void LTHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.LTHeld(id);
    }
    public void DUpHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DUpHeld(id);
    }
    public void DDownHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DDownHeld(id);
    }
    public void DLeftHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DLeftHeld(id);
    }
    public void DRightHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DRightHeld(id);
    }
    public void LJSHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.LJSHeld(id);
    }
    public void RJSHeld() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RJSHeld(id);
    }

    //Released
    public void AReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.AReleased(id);
    }
    public void BReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.BReleased(id);
    }
    public void XReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.XReleased(id);
    }
    public void YReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.YReleased(id);
    }
    public void RBReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RBReleased(id);
    }
    public void LBReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.LBReleased(id);
    }
    public void RTReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RTReleased(id);
    }
    public void LTReleased() {
        for (ControllerInputListener inputListener : listeners)
            inputListener.LTReleased(id);
    }
    public void DUpReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DUpReleased(id);
    }
    public void DDownReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DDownReleased(id);
    }
    public void DLeftReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DLeftReleased(id);
    }
    public void DRightReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.DRightReleased(id);
    }
    public void LJSReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.LJSReleased(id);
    }
    public void RJSReleased() {
        for (ControllerInputListener controllerInput : listeners)
            controllerInput.RJSReleased(id);
    }

    public void Loop(){
        //DETECT EVENTS
        //Pressed
        if(gamepad.a == true && ADown == false) APressed();
        if(gamepad.b == true && BDown == false) BPressed();
        if(gamepad.x == true && XDown == false) XPressed();
        if(gamepad.y == true && YDown == false) YPressed();
        if(gamepad.left_bumper == true && LBDown == false) LBPressed();
        if(gamepad.right_bumper == true && RBDown == false) RBPressed();
        if(gamepad.left_trigger > TriggerThreshold && LTDown == false) LTPressed();
        if(gamepad.right_trigger > TriggerThreshold && RTDown == false) RTPressed();
        if(gamepad.dpad_up == true && DUpDown == false) DUpPressed();
        if(gamepad.dpad_down == true && DDownDown == false) DDownPressed();
        if(gamepad.dpad_left == true && DLeftDown == false) DLeftPressed();
        if(gamepad.dpad_right == true && DRightDown == false) DRightPressed();
        if(gamepad.left_stick_button == true && LJSDown == false) LJSPressed();
        if(gamepad.right_stick_button == true && RJSDown == false) RJSPressed();

        //Held
        if(gamepad.a == true) AHeld();
        if(gamepad.b == true) BHeld();
        if(gamepad.x == true) XHeld();
        if(gamepad.y == true) YHeld();
        if(gamepad.left_bumper == true) LBHeld();
        if(gamepad.right_bumper == true) RBHeld();
        if(gamepad.left_trigger > TriggerThreshold) LTHeld();
        if(gamepad.right_trigger > TriggerThreshold) RTHeld();
        if(gamepad.dpad_up == true) DUpHeld();
        if(gamepad.dpad_down == true) DDownHeld();
        if(gamepad.dpad_left == true) DLeftHeld();
        if(gamepad.dpad_right == true) DRightHeld();
        if(gamepad.left_stick_button == true) LJSHeld();
        if(gamepad.right_stick_button == true) RJSHeld();

        //Released
        if(gamepad.a == false && ADown == true) AReleased();
        if(gamepad.b == false && BDown == true) BReleased();
        if(gamepad.x == false && XDown == true) XReleased();
        if(gamepad.y == false && YDown == true) YReleased();
        if(gamepad.left_bumper == false && LBDown == true) LBReleased();
        if(gamepad.right_bumper == false && RBDown == true) RBReleased();
        if(gamepad.left_trigger <= TriggerThreshold && LTDown == true) LTReleased();
        if(gamepad.right_trigger <= TriggerThreshold && RTDown == true) RTReleased();
        if(gamepad.dpad_up == false && DUpDown == true) DUpReleased();
        if(gamepad.dpad_down == false && DDownDown == true) DDownReleased();
        if(gamepad.dpad_left == false && DLeftDown == true) DLeftReleased();
        if(gamepad.dpad_right == false && DRightDown == true) DRightReleased();
        if(gamepad.left_stick_button == false && LJSDown == true) LJSReleased();
        if(gamepad.right_stick_button == false && RJSDown == true) RJSReleased();


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
