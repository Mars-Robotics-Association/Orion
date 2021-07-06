package org.firstinspires.ftc.teamcode._RobotCode.Oppertunity;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class KenobiWobbleGoalController {

    CRServo LiftServo;
    Servo LeftArm;
    Servo RightArm;

    //  \/These you CAN change.\/
    private final double LiftMaxSpeed = 6;
    private double LiftPower = 0;
    private double LiftDirection = 0;//<-- Changing this one won't do anything

    private final double LeftWobbleGrab = 40;
    private final double LeftRingGrab = 20;

    private final double RightWobbleGrab = 20;
    private final double RightRingGrab = 40;

    //  \/Don't mess with this area.\/
    //DigitalChannel LowerSensor;

    //DigitalChannel UpperSensor;

    public void Init(){
        //LiftServo = setLiftServo;
      //  LeftArm = setLeftArm;
       // RightArm = setRightArm;

        /*
            LowerSensor = hardwareMap.get(DigitalChannel.class, "lowersensor_digital");
            LowerSensor.setMode(DigitalChannel.Mode.INPUT);
            UpperSensor = hardwareMap.get(DigitalChannel.class, "uppersensor_digital");
            UpperSensor.setMode(DigitalChannel.Mode.INPUT);
        */


    }

    public void SetWobbleLiftPower(double power){
        LiftPower = power;
        LiftDirection = 0;
    }
    //Simple setter function for the lift arm. Can be used to force stop raising and lowering.

    public void ChangeWobbleLiftPower(double power){
        LiftPower += power;
        LiftDirection = 0;
    }
    //Works like the Scratch "change" function. Can also be used to force stop raising and lowering.

    public void RaiseWobbleLift(){
        LiftDirection = 1;
        LiftPower = LiftMaxSpeed;
    }
    //Automatically raises the lift to the top. Stopped by SetWobbleLiftPower().

    public void LowerWobbleLift(){
        LiftDirection = -1;
        LiftPower = LiftMaxSpeed;
    }
    //Automatically lowers the lift to the bottom. Stopped by SetWobbleLiftPower().

    public void GrabRing() {

      //  LeftArm.setPosition(LeftRingGrab / 180);
       // RightArm.setPosition(RightRingGrab / 180);

    }
    //Grabs a Ring.

    public void GrabWobbleGoal() {

       // LeftArm.setPosition(LeftWobbleGrab / 180);
     //   RightArm.setPosition(RightWobbleGrab / 180);

    }
    //Grabs a Wobble Goal.

    public void ReleaseArm(){
         //   LeftArm.setPosition(0.5);
         //   RightArm.setPosition(0.5);
    }
    /*
        Releases the Ring or Wobble Goal.
    */

    public void start() {
        ReleaseArm();
    }

    public void Loop() {
        /*
        boolean LowerPressed = !LowerSensor.getState(), UpperPressed = !UpperSensor.getState();
        if(UpperPressed||LowerPressed){
            LiftServo.setSpeed(0);
        } else{
            LiftServo.setSpeed(LiftPower);
        }
        if(LowerPressed && LiftDirection === -1){
            //Stops at the bottom
            LiftDirection = 0;
        } else if(UpperPressed && LiftDirection === 1){
            //Stops at the top
            LiftDirection = 0;
        } else if((UpperPressed||LowerPressed) && LiftDirection !== 0){
            //Flips direction of motor in case it was installed incorrectly
            LiftMaxSpeed *= -1;
            LiftDirection = 0;
        }
        */

    }
}