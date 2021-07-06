package org.firstinspires.ftc.teamcode._RobotCode.Oppertunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.Input.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.Input.ControllerInputListener;

//import org.firstinspires.ftc.teamcode.Robots.Curiosity.CuriosityUltimateGoalControl;

import org.firstinspires.ftc.teamcode.Core.BaseRobots.MecanumBaseControl;

//SimpleTeleOpKenobi (or whatever name is here) shows up on the phone
@TeleOp(name = "SimpleTeleOpKenobi", group = "Competition")

@Config
//This name must match the file name
public class SimpleTeleOp extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    // private WobbleGoalController wobble;
    private MecanumBaseControl control;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    //declare wheel motors
    private DcMotor FR;//front right wheel
    private DcMotor FL;//front left wheel
    private DcMotor RR;//rear right wheel
    private DcMotor RL;//rear left wheel

    private DcMotor shooterLeft;
    private DcMotor shooterRight;


    private CRServo intakeServo;
    private CRServo feedServo;

    // private CRServo wobbleLiftServo;//made the lift a servo instead of a CRServo
    private Servo wobbleLiftServo;//making the lift a servo
    private Servo wobbleLeftServo;
    private Servo wobbleRightServo;


    //private CRServo wobbleCR;
    // private Servo leftServo;
    // private Servo rightServo;


    //   private Servo liftservo;
    //  private Servo leftarm;
    //  private Servo rightarm;


    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = 1;//used to change how fast robot turns
    public static double turnWhileDrivingSpeed = 0.5;

    private double speedMultiplier = 1;

    private boolean busy = false;//used to test if it's ok to drive
    private double turnOffset = 0;

    private int payloadController = 2;

    private double ArmMultiplier = 1;
    private boolean ArmPos = false;
    private boolean ArmNeg = false;

    @Override
    public void init() {
        //map the wheels to the robot configuration
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        RR = hardwareMap.dcMotor.get("RR");
        RL = hardwareMap.dcMotor.get("RL");

        shooterLeft = hardwareMap.dcMotor.get("shooterLeft");
        shooterRight = hardwareMap.dcMotor.get("shooterRight");

        //map the wobble lift and the wobble arms to the robot configuration
        // wobbleLiftServo = hardwareMap.crservo.get("wobbleLiftServo");
        wobbleLiftServo = hardwareMap.servo.get("wobbleLiftServo");//try making lift a servo
        wobbleLeftServo = hardwareMap.servo.get("wobbleLeftServo");
        wobbleRightServo = hardwareMap.servo.get("wobbleRightServo");

        //map the intake, feed, and shooter to the robot configuration
        intakeServo = hardwareMap.crservo.get("intakeServo");
        feedServo = hardwareMap.crservo.get("feedServo");

        //You must set these booleans to use the mecanum wheels: set useChassis to true, usePayload to false, useNavigator to false
        control = new MecanumBaseControl(this, true, false, false);
        control.InitCoreRobotModules();

        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2.addListener(this);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();
    }

    @Override


    public void start(){
        control.StartCoreRobotModules();
        //wobble.start();
    }


    @Override
    public void loop() {


        controllerInput1.Loop();
        controllerInput2.Loop();

        if(!busy) {
            MangeDriveMovement();
        }

        // if(LeftBumper){ArmDirection++;}
        // if(RightBumper){ArmDirection--;}
        //wobble.SetWobbleLiftPower(ArmDirection*ArmMultiplier);

        int dir = 0;

        if(ArmPos){dir++;}
        if(ArmNeg){dir--;}

       //This code will turn the wheels (simple forward and backward - no use of mechanum functions
        //private void ManageDriving() {
        //  double moveX = -gamepad1.left_stick_y*driveSpeed*speedMultiplier;
        //  double moveY = -gamepad1.left_stick_x*driveSpeed*speedMultiplier;
        //   double turn = gamepad1.right_stick_x*turnSpeed*speedMultiplier + turnOffset;


//GAMEPAD 1

        //if the "A" button on gamepad 1 is pressed, the power on the intakeServo is set to 1 (max power)
        if (gamepad1.a) {
            intakeServo.setPower(1);
        }

        if (gamepad1.x) {
            shooterLeft.setPower(-1);//reverses direction
            shooterRight.setPower(1);
        }

        if (gamepad1.b) {
            shooterLeft.setPower(1);
            shooterRight.setPower(-1);//reverses direction
        } else {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }
        if (gamepad1.left_bumper) {
            intakeServo.setPower(1);

        } else {
            intakeServo.setPower(0);
        }

        if (gamepad1.left_trigger > .5) {
            intakeServo.setPower(-1);
        } else {
            intakeServo.setPower(0);
        }

        if (gamepad1.right_bumper) {
            feedServo.setPower(1);
        } else {
            feedServo.setPower(0);
        }

        if (gamepad1.right_trigger > .5) {
            feedServo.setPower(-1);
        } else {
            feedServo.setPower(0);
        }


//GAMEPAD 2
        if (gamepad2.y) {
            wobbleLeftServo.setPosition(0);
            wobbleRightServo.setPosition(1);
        }

        if (gamepad2.b) {
            wobbleLeftServo.setPosition(1);
            wobbleRightServo.setPosition(0);//reverses the direction of the arm
        }
        if (gamepad2.a) {
            wobbleLeftServo.setPosition(.8);
            wobbleRightServo.setPosition(.2);
        }

        //    if (gamepad2.dpad_up) {
        //      wobbleLiftServo.setPower(1);
        //   }
        //   if (gamepad2.dpad_down) {
        //       wobbleLiftServo.setPower(-1);
//
        //  }
        //  if (gamepad2.dpad_up == false && gamepad2.dpad_down == false) {
        //      wobbleLiftServo.setPower(0);
        //  }
        //
        //MAKE THE LIFT A SERVO INSTEAD OF A CRSERVO
        if (gamepad2.dpad_up) {
            wobbleLiftServo.setPosition(.35);
        }
        if (gamepad2.dpad_down) {
            wobbleLiftServo.setPosition(0);
        }

        if (gamepad2.dpad_left) {
            wobbleLiftServo.setPosition(.1);
        }


        // if (gamepad2.dpad_up == false && gamepad2.dpad_down == false) {
        //   wobbleLiftServo.setPosition(.25);
        //}


    }
    //}



//        if(gamepad2.left_stick_y>0.5)
//
//        FL.setPower(gamepad2.left_stick_y);
//        RL.setPower(gamepad2.left_stick_y);
//
//
//        FR.setPower(gamepad2.right_stick_y);
//        RR.setPower(gamepad2.right_stick_y);
//
//        if(gamepad2.left_stick_y<-0.5)
//
//            FL.setPower(gamepad2.left_stick_y);
//            RL.setPower(gamepad2.left_stick_y);
//
//
//            FR.setPower(gamepad2.right_stick_y);
//            RR.setPower(gamepad2.right_stick_y);
//
//




    //FL.setPower(gamepad1.left_stick_y);


    //control.GetOrion().MoveRaw(moveX, moveY, turn);
    //}

    @Override
    public void APressed(double controllerNumber) {
        if(controllerNumber == 1) {
            if (speedMultiplier == 1) speedMultiplier = 0.5;
            else if (speedMultiplier == 0.5) speedMultiplier = 0.25;
            else speedMultiplier = 1;
        }
    }

    @Override
    public void BPressed(double controllerNumber) {
        if(controllerNumber == 1){
            // wobble.RaiseWobbleLift();
        }
    }

    @Override
    public void XPressed(double controllerNumber) {
        if(controllerNumber == 1){
            //  wobble.LowerWobbleLift();
        }
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
    public void BReleased(double controllerNumber)  {

    }

    @Override
    public void XReleased(double controllerNumber) {
    }

    @Override
    public void YReleased(double controllerNumber) {
    }

    @Override
    public void LBPressed(double controllerNumber) {
        if(controllerNumber == 1) {ArmPos = true;}
    }

    @Override
    public void RBPressed(double controllerNumber) {
        if(controllerNumber == 1) {ArmPos = true;}
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
        if(controllerNumber == 1) {ArmPos = false;}
    }

    @Override
    public void RBReleased(double controllerNumber) {
        if(controllerNumber == 1) {ArmNeg = false;}
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

    private void MangeDriveMovement(){
        //MOVE if left joystick magnitude > 0.1
        if (controllerInput1.CalculateLJSMag() > 0.1) {
            control.RawDrive(controllerInput1.CalculateLJSAngle(), controllerInput1.CalculateLJSMag() * driveSpeed, controllerInput1.GetRJSX() * turnWhileDrivingSpeed);//drives at (angle, speed, turnOffset)
            //control.GetOrion().MoveRaw(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed, controllerInput1.GetRJSX()*turnWhileDrivingSpeed);
            telemetry.addData("Moving at ", controllerInput1.CalculateLJSAngle());
        }
        //TURN if right joystick magnitude > 0.1 and not moving
        else if (Math.abs(controllerInput1.GetRJSX()) > 0.1) {
            control.RawTurn(controllerInput1.GetRJSX() * turnSpeed *-1);//turns at speed according to rjs1
            //control.GetOrion().TurnRaw(controllerInput1.GetRJSX() * turnSpeed);
            telemetry.addData("Turning", true);
        }
        else {
            control.GetChassis().SetMotorSpeeds(0, 0, 0, 0);
        }
    }
}
