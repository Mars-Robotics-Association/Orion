package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.BaseRobots.MecanumBaseControl;

/**
 * Control class for the Belinda Robot. Controls payload.
 * Required to run: Phones | REV Hub | Belinda Chassis
 * Suggested to run: Shooter | Intake | Odometry | Webcam
 */
//The class used to control the demobot. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the demobot.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit

@Config
public class CuriosityUltimateGoalControl extends MecanumBaseControl
{
    ////Dependencies////
    //Mechanical Components
    private CuriosityPayloadController payload;

    ////Variables////
    //Calibration
    public static double shootXOffset = 70;
    public static double shootYOffset = 0;
    public static double shootAngleOffset = 180;

    /**@param setOpMode pass the opmode running this down to access hardware map
     * @param useChassis whether to use the chassis of the robot
     * @param usePayload whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public CuriosityUltimateGoalControl(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, useChassis, usePayload, useNavigator);
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===
        if(USE_PAYLOAD) {
            DcMotor shooterMotor1 = currentOpMode.hardwareMap.dcMotor.get("SM1");
            DcMotor shooterMotor2 = currentOpMode.hardwareMap.dcMotor.get("SM2");
            Servo intakeServo1 = currentOpMode.hardwareMap.servo.get("intakeServo1");
            Servo intakeServo2 = currentOpMode.hardwareMap.servo.get("intakeServo2");
            Servo starpathServo = currentOpMode.hardwareMap.servo.get("starpathServo");

            DistanceSensor intakeSensor = currentOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "intake sensor");

            payload = new CuriosityPayloadController(currentOpMode, new DcMotor[]{shooterMotor1, shooterMotor2}, intakeServo1, intakeServo2, starpathServo, intakeSensor);
        }

        //TODO ===INIT CORE ROBOT===
        super.InitCoreRobotModules();

        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){
        super.StartCoreRobotModules();
    }

    //CALLABLE METHODS//
    public void ShootOne(){ payload.ShootOne();}

    public void ShootRoutine(){
        //TODO: fill in
    }

    public void AlignAndShoot(){
        ShooterOn();
        orion.AlignToVumark(0, shootXOffset, shootYOffset, shootAngleOffset);
        ShootThree();
    }

    public void ShootThree(){ payload.ShootThree();}
    public void StopShootThree(){ payload.StopShooter();}

    public void ShooterOn(){ payload.ShooterOn();}
    public void ShooterOff(){ payload.ShooterOff();}
    public void ToggleShooterMotors(){payload.ToggleShooterMotors();}

    public void TurnToZero(){orion.TurnTo(0);}
    public void SetHome(){orion.SetPose(0,0,0);}
    public void GoToHome(){
        orion.MoveLine(0,0,0);
    }

    public void Powershot(){
        orion.SetOriginToVumark(0);
        orion.MoveLine(75,-20,0);
        orion.TurnTo(0);
        ShootOne();
        orion.TurnTo(-5);
        ShootOne();
        orion.TurnTo(10);
        ShootOne();
    }

    public void ModifyForPowerShot(){ payload.ModifyForPowerShot();}
    public void StopModifyForPowerShot(){ payload.StopModifyForPowerShot();}

    public void IntakeOn(){ payload.IntakeOn();}
    public void IntakeLoop(){ payload.IntakeLoop();}
    public void StopIntake(){ payload.IntakeOff();}
    public void ReverseIntake(){ payload.ReverseIntake();}
    public void ToggleIntaking(){payload.ToggleIntaking();}

    public void RotateStarpathToNextPos(){ payload.RotateStarpathToNextPos();}
    public void RotateStarpathToPreviousPos(){ payload.RotateStarpathToPreviousPos();}
    public void StarpathToShooter(){ payload.StarPathToShooter();}
    public void StarpathToIntake(){ payload.StarpathToIntake();}
    public void StarpathToStorage(){payload.StarpathToStorage();}
}
