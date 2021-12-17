package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults.DefaultNavProfile;

@TeleOp(name = "Andrew TeleOp", group = "All")
public class AndrewTeleop extends OpMode implements ControllerInputListener {

    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;
    private MecanumChassis mecanumChassis;

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RR;
    private DcMotor RL;
    private CRServo duckyServo;
    private boolean backWasDown = false;

    private double speed = 1;


    public void init() {
        controllerInput1 = new ControllerInput(gamepad1,1);
        controllerInput2 = new ControllerInput(gamepad2,2);
        FR = this.hardwareMap.dcMotor.get("FR");
        FL = this.hardwareMap.dcMotor.get("FL");
        RR = this.hardwareMap.dcMotor.get("RR");
        RL = this.hardwareMap.dcMotor.get("RL");
        duckyServo = this.hardwareMap.crservo.get("duckyServo");
    }
        public void start(){

        }


        public void loop(){
            controllerInput1.Loop();
            controllerInput2.Loop();

            double stickDir = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double stickDist = Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2));
            double[] newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(stickDir*180/3.14-90,stickDist,0-gamepad1.right_stick_x);
            for(int i = 0; i<4; i++)
                newSpeeds[i]*=speed;

            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[1]);
            RR.setPower(newSpeeds[2]);
            RL.setPower(newSpeeds[3]);

            if(gamepad1.x)
                duckyServo.setPower(0-speed);
            else if(gamepad1.b)
                duckyServo.setPower(speed);
            else
                duckyServo.setPower(0);
            

            if(gamepad1.back&&!backWasDown){
                if(speed==1)
                    speed = 0.5;
                else
                    speed=1;
            }
            backWasDown = gamepad1.back;

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

