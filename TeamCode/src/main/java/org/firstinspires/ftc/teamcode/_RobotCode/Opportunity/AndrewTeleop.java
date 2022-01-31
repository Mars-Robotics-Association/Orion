package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults.DefaultNavProfile;

@TeleOp(name = "Andrew TeleOp", group = "All")
@Disabled
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
    private DcMotor gripper;
    private DcMotor turntable;
    private DistanceSensor sideDist;

    private ColorSensor colorSensor1;

    private TouchSensor armZeroTouchSensor;
   // private TouchSensor armTouch;
    private DigitalChannel armTouch;

    private IMU imu;
    private AndrewIMU andrewIMU;
   // private CRServo duckyServo;
    private DcMotor duckyMotor;
    private boolean backWasDown = false;
    private boolean startWasDown = false;
    private boolean lBumperWasDown = false;
    private boolean dpad2WasDown = false;

    private boolean doFunnyLockThing = false;

    private double speed = 1;
    private int gripperStart = 0;


    private int armStartPos = 0;

    private AndrewArm andrewArm;


    public void init() {
        controllerInput1 = new ControllerInput(gamepad1,1);
        controllerInput2 = new ControllerInput(gamepad2,2);
        FR = this.hardwareMap.dcMotor.get("FR");
        FL = this.hardwareMap.dcMotor.get("FL");
        RR = this.hardwareMap.dcMotor.get("RR");
        RL = this.hardwareMap.dcMotor.get("RL");
        gripper = this.hardwareMap.dcMotor.get("clawMotor");
        turntable = this.hardwareMap.dcMotor.get("turntable");

        sideDist = hardwareMap.get(DistanceSensor.class, "distSide");
  //      colorSensor1 = hardwareMap.colorSensor.get("color2");
    //    RL.setDirection(DcMotorSimple.Direction.REVERSE); //uncomment this too
    //    FR.setDirection(DcMotorSimple.Direction.REVERSE);


        armPos = this.hardwareMap.dcMotor.get("armPosition");
    //    duckyServo = this.hardwareMap.crservo.get("duckyServo");
        duckyMotor = this.hardwareMap.dcMotor.get("duckyMotor");
        //armTouch = this.hardwareMap.get(TouchSensor.class,"armTouch");

        armTouch = hardwareMap.get(DigitalChannel.class, "armTouch");
        armTouch.setMode(DigitalChannel.Mode.INPUT);

        imu = new IMU(this);
        andrewIMU = new AndrewIMU(imu);
    }
        public void start(){
            imu.Start();
            andrewArm = new AndrewArm(armPos);
            andrewIMU.resetRotation();
            gripperStart = gripper.getCurrentPosition();
            armStartPos = armPos.getCurrentPosition();
        }


        public void loop(){

        telemetry.addData("Distance sensor",sideDist.getDistance(DistanceUnit.INCH));

            controllerInput1.Loop();
            controllerInput2.Loop();
            andrewIMU.loop();
            andrewArm.loop();

            double stickDir = Math.PI+Math.atan2(gamepad1.left_stick_y, 0-gamepad1.left_stick_x);
            double stickDist = Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2));

            double moveAngle = stickDir*180/3.14;
            if(doFunnyLockThing) moveAngle = ((stickDir*180/3.14)- andrewIMU.getRotation());
            double[] newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(moveAngle,stickDist,gamepad1.right_stick_x);
            telemetry.addData("driveAngle",moveAngle);
            for(int i = 0; i<4; i++)
                newSpeeds[i]*=speed;

            FR.setPower(newSpeeds[0]); //uncoment this!!
            FL.setPower(newSpeeds[2]);
            RR.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);

//            if(gamepad1.a)
//                FR.setPower(1);
//            else
//                FR.setPower(0);
//
//            if(gamepad1.b)
//                FL.setPower(1);
//            else
//                FL.setPower(0);
//
//            if(gamepad1.x)
//                RR.setPower(1);
//            else
//                RR.setPower(0);
//
//            if(gamepad1.y)
//                RL.setPower(1);
//            else
//                RL.setPower(0);

            if(gamepad1.x)
                duckyMotor.setPower(1);
            else if(gamepad1.b)
                duckyMotor.setPower(0-1);
            else
                duckyMotor.setPower(0);

            if(gamepad1.left_bumper&&!backWasDown){
                if(speed==1)
                    speed = 0.5;
                else
                    speed=1;
            }
            backWasDown = gamepad1.left_bumper;


            if(gamepad1.start&&!startWasDown){
                doFunnyLockThing = !doFunnyLockThing;
                andrewIMU.resetRotation();
            }
            startWasDown = gamepad1.start;



            player2ArmStuff();




//            telemetry.addData("FR",FR.getCurrentPosition());
//            telemetry.addData("FL",FL.getCurrentPosition());
//            telemetry.addData("RR",RR.getCurrentPosition());
//            telemetry.addData("RL",RL.getCurrentPosition());
//          //  telemetry.update();
//



        }
        private boolean gamepad2APressed = false;
        private boolean gripperToggle;
        private int turntableLockPos = 0;
        private boolean turntableLockOn = false;


        public void player2ArmStuff(){
            telemetry.addData("gripper position",gripper.getCurrentPosition()+gripperStart);


            /*
        if(Math.abs(gamepad2.left_stick_y)>0.1) //ARM UP/DOWN
            armPos.setPower(gamepad2.left_stick_y);
        else
            armPos.setPower(0);
*/



/*
        if(Math.abs(gamepad2.right_stick_x)>0.1) //TURNTABLE
            turntable.setPower(0-gamepad2.right_stick_x);
        else
            turntable.setPower(0);
*/
            if(gamepad1.dpad_left||gamepad2.dpad_left) {

                turntable.setPower(1);
            }else if(gamepad1.dpad_right||gamepad2.dpad_right) {
                turntable.setPower(0 - 1);
            }else{

                  turntable.setPower(0);


            }



            if(gamepad1.dpad_up||gamepad2.dpad_up)
                armPos.setPower(-1);
            else if(gamepad1.dpad_down||gamepad2.dpad_down)
                armPos.setPower(1);
            else
                armPos.setPower(0);


        if(!gamepad2APressed&&(gamepad1.a||gamepad2.a)){ //GRIPPER TOGGLE
            gripperToggle = !gripperToggle;
        }
        gamepad2APressed = (gamepad1.a||gamepad2.a);
        if(gripperToggle){
          gripperStart = gripper.getCurrentPosition();
          gripper.setTargetPosition(400+gripperStart);
          if(gripper.getCurrentPosition()+gripperStart>300)
              gripper.setPower(0.2);
          else
              gripper.setPower(1);
          gripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      }else{
          gripper.setPower(0);
      }





        }






    public void things(){

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

            */
        if(gamepad2.back){
            andrewArm.zeroArm();
            andrewArm.setRawPower(0);
        }

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


        if(Math.abs(gamepad2.left_stick_y)>0.1){
            andrewArm.setAdjustmentSpeed(0-gamepad2.left_stick_y);
        }else{
            andrewArm.setAdjustmentSpeed(0);
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

