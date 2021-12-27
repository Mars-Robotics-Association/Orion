package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults.DefaultNavProfile;

@TeleOp(name = "Andrew TeleOp", group = "All")
public class AndrewTeleop extends OpMode implements ControllerInputListener {

    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;
    private MecanumChassis mecanumChassis;

    private String[] david = "version 1... And you may find yourself living in a shotgun shack;And you may find yourself in another part of the world;And you may find yourself behind the wheel of a large automobile;And you may find yourself in a beautiful house, with a beautiful wife;And you may ask yourself, \"Well, how did I get here?\";Letting the days go by, let the water hold me down;Letting the days go by, water flowing underground;Into the blue again after the money's gone;Once in a lifetime, water flowing underground;And you may ask yourself, \"How do I work this?\";And you may ask yourself, \"Where is that large automobile?\";And you may tell yourself, \"This is not my beautiful house\";And you may tell yourself, \"This is not my beautiful wife\";Letting the days go by, let the water hold me down;Letting the days go by, water flowing underground;Into the blue again after the money's gone;Once in a lifetime, water flowing underground;Same as it ever was, same as it ever was;Same as it ever was, same as it ever was;Same as it ever was, same as it ever was;Same as it ever was, same as it ever was;Water dissolving and water removing;There is water at the bottom of the ocean;Under the water, carry the water;Remove the water at the bottom of the ocean;Water dissolving and water removing;Letting the days go by, let the water hold me down;Letting the days go by, water flowing underground;Into the blue again, into the silent water;Under the rocks and stones, there is water underground;Letting the days go by, let the water hold me down;Letting the days go by, water flowing underground;Into the blue again after the money's gone;Once in a lifetime, water flowing underground;You may ask yourself, \"What is that beautiful house?\";You may ask yourself, \"Where does that highway go to?\";And you may ask yourself, \"Am I right? Am I wrong?\";And you may say to yourself, \"My God! What have I done?\";Letting the days go by, let the water hold me down;Letting the days go by, water flowing underground;Into the blue again, into the slent water;Under the rocks and stones, there is water underground;Letting the days go by, let the water hold me down;Letting the days go by, water flowing underground;Into the blue again after the money's gone;Once in a lifetime, water flowing underground;Same as it ever was, same as it ever was;Same as it ever was and look where my hand was;Time isn't holding up, time isn't after us;Same as it ever was, same as it ever was;Same as it ever was, same as it ever was;Same as it ever was, same as it ever was;Letting the days go by, same as it ever was;Here a twister comes, here comes the twister;Same as it ever was, same as it ever was (Letting the days go by);Same as it ever was, same as it ever was (Letting the days go by);Once in a lifetime (Let the water hold me down);Letting the days go by (Water flowing underground);Into the blue again".split(";");
    private int davidIndex = 0;

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RR;
    private DcMotor RL;
    private IMU imu;
    private AndrewIMU andrewIMU;
    private CRServo duckyServo;
    private boolean backWasDown = false;
    private boolean startWasDown = false;
    private boolean lBumperWasDown = false;

    private boolean doFunnyLockThing = false;

    private double speed = 1;


    public void init() {
        controllerInput1 = new ControllerInput(gamepad1,1);
        controllerInput2 = new ControllerInput(gamepad2,2);
        FR = this.hardwareMap.dcMotor.get("FR");
        FL = this.hardwareMap.dcMotor.get("FL");
        RR = this.hardwareMap.dcMotor.get("RR");
        RL = this.hardwareMap.dcMotor.get("RL");
        duckyServo = this.hardwareMap.crservo.get("duckyServo");
        imu = new IMU(this);
        andrewIMU = new AndrewIMU(imu);
    }
        public void start(){
            imu.Start();
        }


        public void loop(){


            controllerInput1.Loop();
            controllerInput2.Loop();
            andrewIMU.loop();

            telemetry.addData("angle",andrewIMU.getRotation());

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

            if(gamepad2.left_bumper&&!lBumperWasDown){
                if(davidIndex>=david.length)
                    davidIndex = 0;
                telemetry.speak(""+david[davidIndex]);
                davidIndex++;
            }

            lBumperWasDown = gamepad2.left_bumper;

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

