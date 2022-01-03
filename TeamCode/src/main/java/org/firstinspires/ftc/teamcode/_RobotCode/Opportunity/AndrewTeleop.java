package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults.DefaultNavProfile;

@TeleOp(name = "Andrew TeleOp", group = "All")
//@Disabled
public class AndrewTeleop extends OpMode implements ControllerInputListener {

    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;
    private MecanumChassis mecanumChassis;

    private int armPosIndex = 1;
    private double[] armPositions = {-0.15,0,0.5,0.7,1,1.1,1.3};

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RR;
    private DcMotor RL;
    private DcMotor armPos;

    private TouchSensor armZeroTouchSensor;
   // private TouchSensor armTouch;
    private DigitalChannel armTouch;

    private IMU imu;
    private AndrewIMU andrewIMU;
    private CRServo duckyServo;
    private boolean backWasDown = false;
    private boolean startWasDown = false;
    private boolean lBumperWasDown = false;
    private boolean dpad2WasDown = false;

    private boolean doFunnyLockThing = false;

    private double speed = 1;

    private double armSpeed = 0.5;

    private double armAngle = 0;
    private AndrewArm andrewArm;


    public void init() {
        controllerInput1 = new ControllerInput(gamepad1,1);
        controllerInput2 = new ControllerInput(gamepad2,2);
        FR = this.hardwareMap.dcMotor.get("FR");
        FL = this.hardwareMap.dcMotor.get("FL");
        RR = this.hardwareMap.dcMotor.get("RR");
        RL = this.hardwareMap.dcMotor.get("RL");
        armPos = this.hardwareMap.dcMotor.get("armPosition");
        duckyServo = this.hardwareMap.crservo.get("duckyServo");
        //armTouch = this.hardwareMap.get(TouchSensor.class,"armTouch");

        armTouch = hardwareMap.get(DigitalChannel.class, "armTouch");
        armTouch.setMode(DigitalChannel.Mode.INPUT);

        imu = new IMU(this);
        andrewIMU = new AndrewIMU(imu);
    }
        public void start(){
            imu.Start();
            andrewArm = new AndrewArm(armPos);
        }


        public void loop(){


            controllerInput1.Loop();
            controllerInput2.Loop();
            andrewIMU.loop();
            andrewArm.loop();



            double stickDir = Math.atan2(gamepad1.left_stick_y, 0-gamepad1.left_stick_x);
            double stickDist = Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2));

            double moveAngle = stickDir*180/3.14;
            if(doFunnyLockThing) moveAngle = (stickDir*180/3.14)- andrewIMU.getRotation();
            double[] newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(moveAngle,stickDist,0-gamepad1.right_stick_x);

            for(int i = 0; i<4; i++)
                newSpeeds[i]*=speed;

            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[1]);
            RR.setPower(newSpeeds[2]);
            RL.setPower(newSpeeds[3]);

            if(gamepad1.x)
                duckyServo.setPower(1);
            else if(gamepad1.b)
                duckyServo.setPower(0-1);
            else
                duckyServo.setPower(0);

            if(gamepad1.back&&!backWasDown){
                if(speed==1)
                    speed = 0.5;
                else
                    speed=1;
            }
            backWasDown = gamepad1.back;


            if(gamepad1.start&&!startWasDown){
                doFunnyLockThing = !doFunnyLockThing;
                andrewIMU.resetRotation();
            }
            startWasDown = gamepad1.start;


            if(gamepad2.x){
                andrewArm.setRawPower(0.5);
            }
            if(gamepad2.y){
                andrewArm.setRawPower(-0.5);
            }
            /*
            if(gamepad2.a){
                andrewArm.setTarget(0.5,1,0.03);
            }
            if(gamepad2.b){
                andrewArm.setTarget(0,1,0.03);
            }
            if(gamepad2.back){
                andrewArm.zeroArm();
            }
            */

            boolean indexWasChanged = false;

            if(gamepad2.dpad_left&&!dpad2WasDown){
                armPosIndex--;
                indexWasChanged= true;
            }
            if(gamepad2.dpad_right&&!dpad2WasDown){
                armPosIndex++;
                indexWasChanged= true;
            }
            if(gamepad2.dpad_up&&!dpad2WasDown){
                armPosIndex = armPositions.length-1;
                indexWasChanged= true;
            }
            if(gamepad2.dpad_down&&!dpad2WasDown){
                armPosIndex = 0;
                indexWasChanged= true;
            }
if(armPosIndex>=armPositions.length) armPosIndex = armPositions.length-1;
if(armPosIndex<0) armPosIndex = 0;




            dpad2WasDown = gamepad2.dpad_up||gamepad2.dpad_down||gamepad2.dpad_left||gamepad2.dpad_right;
            if(dpad2WasDown)  andrewArm.setTarget(armPositions[armPosIndex],1,0.03);

            if(gamepad2.start){
                andrewArm.setRawPower(0);
            }

            if(!armTouch.getState()){
                andrewArm.zeroArm();
                andrewArm.setTarget(0,0.4,0.02);
                armPosIndex = 1;
            }


            armAngle = andrewArm.getAngle();

            telemetry.addData("Arm angle",andrewArm.getAngle());
            telemetry.addData("Arm target",andrewArm.getTarget());
            telemetry.addData("TargetSetCount" , andrewArm.targetCount);
            telemetry.addData("left_stick_y",gamepad2.left_stick_y);

            if(Math.abs(gamepad2.left_stick_y)>0.1){
                andrewArm.setAdjustmentSpeed(0-gamepad2.left_stick_y);
            }else{
                andrewArm.setAdjustmentSpeed(0);
            }

          //  telemetry.update();



        }



    @Override
    public void APressed(double controllerNumber) {

    }

    @Override
    public void BPressed(double controllerNumber) {

    }

    @Override
    public void XPressed(double controllerNumber) {

    }

    @Override
    public void YPressed(double controllerNumber) {

    }

    @Override
    public void AHeld(double controllerNumber) {

    }

    @Override
    public void BHeld(double controllerNumber) {

    }

    @Override
    public void XHeld(double controllerNumber) {

    }

    @Override
    public void YHeld(double controllerNumber) {

    }

    @Override
    public void AReleased(double controllerNumber) {

    }

    @Override
    public void BReleased(double controllerNumber) {

    }

    @Override
    public void XReleased(double controllerNumber) {

    }

    @Override
    public void YReleased(double controllerNumber) {

    }

    @Override
    public void LBPressed(double controllerNumber) {

    }

    @Override
    public void RBPressed(double controllerNumber) {

    }

    @Override
    public void LTPressed(double controllerNumber) {

    }

    @Override
    public void RTPressed(double controllerNumber) {

    }

    @Override
    public void LBHeld(double controllerNumber) {

    }

    @Override
    public void RBHeld(double controllerNumber) {

    }

    @Override
    public void LTHeld(double controllerNumber) {

    }

    @Override
    public void RTHeld(double controllerNumber) {

    }

    @Override
    public void LBReleased(double controllerNumber) {

    }

    @Override
    public void RBReleased(double controllerNumber) {

    }

    @Override
    public void LTReleased(double controllerNumber) {

    }

    @Override
    public void RTReleased(double controllerNumber) {

    }

    @Override
    public void DUpPressed(double controllerNumber) {

    }

    @Override
    public void DDownPressed(double controllerNumber) {

    }

    @Override
    public void DLeftPressed(double controllerNumber) {

    }

    @Override
    public void DRightPressed(double controllerNumber) {

    }

    @Override
    public void DUpHeld(double controllerNumber) {

    }

    @Override
    public void DDownHeld(double controllerNumber) {

    }

    @Override
    public void DLeftHeld(double controllerNumber) {

    }

    @Override
    public void DRightHeld(double controllerNumber) {

    }

    @Override
    public void DUpReleased(double controllerNumber) {

    }

    @Override
    public void DDownReleased(double controllerNumber) {

    }

    @Override
    public void DLeftReleased(double controllerNumber) {

    }

    @Override
    public void DRightReleased(double controllerNumber) {

    }

    @Override
    public void LJSPressed(double controllerNumber) {

    }

    @Override
    public void RJSPressed(double controllerNumber) {

    }

    @Override
    public void LJSHeld(double controllerNumber) {

    }

    @Override
    public void RJSHeld(double controllerNumber) {

    }

    @Override
    public void LJSReleased(double controllerNumber) {

    }

    @Override
    public void RJSReleased(double controllerNumber) {

    }


}

