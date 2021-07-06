package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class CuriosityPayloadController_old
{
    ////DEPENDENCIES////
    OpMode opMode;
    //Motors
    private DcMotor[] shooterMotors;
    //Servos
    private Servo intakeServo1;
    private Servo intakeServo2;
    private Servo bootServo;
    private Servo starpathServo;
    private Servo loaderServo;

    //Sensors
    private DistanceSensor intakeDetector;
    private DistanceSensor shooterDetector;

    ////CONFIG VARIABLES////
    //public static double bootInPos = 0.7;
    //public static double bootOutPos = 0.45;

    public static double starpathDownPos = 0.01;
    public static double starpathIntervalIntake = 0.095;
    public static double starpathIntervalShooter = 0.08;
    public static double starpathUpPos = 0.24;

    //public static double loaderClearPos = 0.65;
    //public static double loaderLoadPos = 1;

    public static double shooterSpeedMultiplier = -1;
    public static double powerShotSpeedMultiplier = 0.9;

    public static double timeToShoot = 0.5;
    public static double timeInitialSpinup = 2;
    //public static double timeToRetract = 0.5;
    //public static double timeToMoveToNext = 0.5;

    public static double moveUpFromIntakeDistCM = 10;
    public static boolean moveUpFromIntakeEnabled = false;

    ////PUBLIC VARIABLES////
    public boolean shooterRunning = false;

    public boolean stopShooterOverride = false;

    ////PRIVATE VARIABLES////
    private double shooterStartTime = 0;
    //private double loaderStartTime = 0;
    //private boolean loadFromIntakeRunning = false;
    private int starpathPosition = 0; //starts at 0, 3 is to the shooter, 5 is max
    private int discsShot = 3;
    //private boolean loaderUsed = false;
    //private boolean starpathUsed = false;
    //private boolean bootUsed = false;
    private boolean discDetectedInIntake = false;
    public boolean powerShot = false;

    public void Init(OpMode setOpMode, DcMotor[] setShooterMotors, Servo setIntakeServo1, Servo setIntakeServo2, Servo setBootServo, Servo setStarpathServo, Servo setLoaderServo, DistanceSensor setIntakeSensor){
        opMode = setOpMode;

        shooterMotors = setShooterMotors;

        intakeServo1 = setIntakeServo1;
        intakeServo2 = setIntakeServo2;
        bootServo = setBootServo;
        starpathServo = setStarpathServo;
        loaderServo = setLoaderServo;

        intakeDetector = setIntakeSensor;
    }

    private void SetShooterPower(double power){
        int i = 0;
        for (DcMotor motor: shooterMotors) {
            motor.setPower(power);
            i++;
        }
    }

    //TODO: remove
    /*private void BootDisc(){bootServo.setPosition(bootInPos);}
    private void BootMiddle(){bootServo.setPosition((bootOutPos+bootInPos)/2);}
    private void BootReset(){bootServo.setPosition(bootOutPos);}*/

    public void StarpathToIntake(){
        starpathServo.setPosition(starpathDownPos);
        discsShot = 3;
        starpathPosition = 0;
    }

    private void StarpathAddIntervalIntake(){starpathServo.setPosition(starpathServo.getPosition()+ starpathIntervalIntake);}
    private void StarpathSubtractIntervalIntake(){starpathServo.setPosition(starpathServo.getPosition()- starpathIntervalIntake);}
    private void StarpathAddIntervalShooter(){starpathServo.setPosition(starpathServo.getPosition()+ starpathIntervalShooter);}
    private void StarpathSubtractIntervalShooter(){starpathServo.setPosition(starpathServo.getPosition()- starpathIntervalShooter);}
    public void StarPathToShooter(){

        starpathServo.setPosition(starpathUpPos);
        discsShot = 0;
        starpathPosition = 3;
    }

    //TODO: remove
    /*private void LoaderClear(){loaderServo.setPosition(loaderClearPos);}
    private void LoaderLoad(){loaderServo.setPosition(loaderLoadPos);}*/

    public void ShooterOn(){
        double modifier = shooterSpeedMultiplier;
        if(powerShot) modifier *= powerShotSpeedMultiplier;
        SetShooterPower(modifier);
    }
    public void ShooterOff(){SetShooterPower(0);}

    public void RotateStarpathToNextPos(){
        //if (intakeDetector.getDistance(DistanceUnit.CM) < 20 && starpathPosition == 5 || shooterDetector.getDistance(DistanceUnit.CM) < 20)
            //return;
        starpathPosition++;

        //if next pos is 6, reset back to 0
        if(starpathPosition == 6){
            StarpathToIntake();
        }
        //if next pos is 3, go to the shooter
        else if(starpathPosition == 3) StarPathToShooter();
        //else, add with the interval
        else if(starpathPosition < 3) StarpathAddIntervalIntake();
        else if(starpathPosition > 3) StarpathAddIntervalShooter();
    }

    public void RotateStarpathToPreviousPos(){
        if(starpathPosition != 0) starpathPosition--;

        if(starpathPosition == 2) starpathServo.setPosition(starpathIntervalIntake *2);
        else if(starpathPosition == 0){
            StarpathToIntake();
        }
        else StarpathSubtractIntervalIntake();
    }

    public void Intake(){
        /*//if starpath not at intake and its shot all discs, return it to intake
        if(starpathPosition > 2 && discsShot == 3){
            StarpathToIntake();
        }*/

        //TODO: remove
        //if(!loadFromIntakeRunning) BootReset();

        //Manages auto intaking
        if(intakeDetector.getDistance(DistanceUnit.CM) < moveUpFromIntakeDistCM && !discDetectedInIntake){
            if(starpathPosition < 2 && moveUpFromIntakeEnabled) StarpathAddIntervalIntake();
            discDetectedInIntake = true;
        }
        if (!(intakeDetector.getDistance(DistanceUnit.CM) < moveUpFromIntakeDistCM)) discDetectedInIntake = false;

        if(starpathPosition > 2){
            StopIntake();
            return;
        }
        discsShot = 0;
        //run intake
        intakeServo1.setPosition(1);
        intakeServo2.setPosition(0);
    }
    public void StopIntake(){
        intakeServo1.setPosition(0.5);
        intakeServo2.setPosition(0.5);
    }

    //TODO: remove
    /*public void LoadFromIntake(){
        //start timer
        if(!loadFromIntakeRunning)loaderStartTime = opMode.getRuntime();
        loadFromIntakeRunning = true;

        //boot disc into starpath
        if(!bootUsed) BootDisc();

        //wait
        if(loaderStartTime+1 > opMode.getRuntime()) return;

        BootReset();

        bootUsed = true;
    }*/
    /*public void StopLoadFromIntake(){
        //rotate starpath to next pos
        if(starpathPosition < 3) RotateStarpathToNextPos();
        bootUsed = false;
        loadFromIntakeRunning = false;
    }*/

    //TODO: remove
    /*public void ShootAsync(){
        //start timer
        if(!shooterRunning)shooterStartTime = opMode.getRuntime();
        shooterRunning = true;

        //start shooter
        ShooterOn();

        //wait
        //if(shooterStartTime+2 > opMode.getRuntime() && discsShot <1) return;

        //load shooter -> it shoots
        if(!loaderUsed){
            //LoaderLoad();
            discsShot ++;
            loaderUsed = true;
            shooterStartTime = opMode.getRuntime();
        }

        //wait
        if(shooterStartTime+timeToShoot > opMode.getRuntime()) return;

        //retract loader
        LoaderClear();

        //wait
        if(shooterStartTime+timeToShoot+timeToRetract > opMode.getRuntime()) return;

        //move to next pos
        if(!starpathUsed && discsShot != 3){
            RotateStarpathToNextPos();
            starpathUsed = true;
        }
        else if(!starpathUsed && discsShot == 3){
            StarpathToIntake();
            starpathUsed = true;
        }

        //wait
        if(shooterStartTime+timeToShoot+timeToRetract+timeToMoveToNext > opMode.getRuntime()) return;
        shooterRunning = false;
    }*/
    /*public void StopShootAsync(){
        ShooterOff();
        LoaderClear();
        shooterRunning = false;
        loaderUsed = false;
        starpathUsed = false;
    }*/

    public void ShootThree(){
        ShooterOn(); //Spin up shooter
        stopShooterOverride = false;

        if(!shooterRunning)shooterStartTime = opMode.getRuntime();
        shooterRunning = true;

        //wait
        while (shooterStartTime+timeInitialSpinup > opMode.getRuntime()) opMode.telemetry.addLine("waiting to spin up...");

        ShootOne();
        ShooterOn();

        ShootOne();
        ShooterOn();

        ShootOne();

        //Reset shooter
        ShooterOff();
    }
    public void StopShooting(){
        stopShooterOverride = true;
        shooterRunning = false;
        ShooterOff();
    }

    public void ShootOne(){
        if(!shooterRunning)shooterStartTime = opMode.getRuntime();
        shooterRunning = true;

        if(stopShooterOverride) return;

        //start shooter
        ShooterOn();
        //wait
        while (shooterStartTime+timeToShoot > opMode.getRuntime()) opMode.telemetry.addLine("waiting to shoot...");
        //fire
        StarpathAddIntervalIntake();
    }

    public void ModifyForPowerShot(){
        powerShot = true;
    }
    public void StopModifyForPowerShot(){
        powerShot = false;
    }
}
