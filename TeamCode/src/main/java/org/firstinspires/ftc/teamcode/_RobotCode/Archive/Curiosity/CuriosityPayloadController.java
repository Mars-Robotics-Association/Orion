package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class CuriosityPayloadController
{
    ////CALIBRATION////
    //Starpath
    public static double starpathIntakePosition = 0.0;
    public static double starpathIntervalIntake = 0.088;
    public static double starpathIntervalShooter = 0.074;
    public static double starpathShooterPosition = 0.25;
    public static double starpathStoragePosition = 0.225;

    //Shooter
    public static double shooterSpeedMultiplier = -1;
    public static double powerShotSpeedMultiplier = 0.85;
    public static double timeToShoot = 1;
    public static double timeInitialSpinup = 1;

    //Auto Intake
    public static double autoIntakeDistanceCM = 5.5;
    public static double autoIntakeCooldown = 1;
    public static boolean autoIntakeEnabled = true;

    ////DEPENDENCIES////
    private OpMode currentOpmode;
    //Motors
    private DcMotor[] shooterMotors;
    //Servos
    private Servo intakeServo1;
    private Servo intakeServo2;
    private Servo starpathServo;
    //Sensors
    private DistanceSensor intakeDetector;
    private DistanceSensor shooterDetector;


    ////PUBLIC////
    public boolean powerShot = false;

    ////PRIVATE////
    private int starpathPosition = 0; //starts at 0, 3 is to the shooter, 5 is max

    private boolean discDetectedInIntake = false;
    private double lastAutoIntakeTime = 0;
    private boolean intakeRunning = false;
    private boolean intakeReverse = false;

    private boolean shootRoutineRunning = false;
    private boolean shooterMotorsRunning = false;
    private boolean stopShooterOverride = false;

    private double shooterStartTime = 0;


    public CuriosityPayloadController(OpMode setOpMode, DcMotor[] setShooterMotors, Servo setIntakeServo1, Servo setIntakeServo2, Servo setStarpathServo, DistanceSensor setIntakeSensor){
        currentOpmode = setOpMode;

        shooterMotors = setShooterMotors;

        intakeServo1 = setIntakeServo1;
        intakeServo2 = setIntakeServo2;
        starpathServo = setStarpathServo;

        intakeDetector = setIntakeSensor;
    }

    ////Public Functions////
    //Shooter
    public void ModifyForPowerShot(){
        powerShot = true;
    }
    public void StopModifyForPowerShot(){
        powerShot = false;
    }

    public void ShooterOn(){
        shooterMotorsRunning = true;
        double modifier = shooterSpeedMultiplier;
        if(powerShot) modifier *= powerShotSpeedMultiplier;
        SetShooterPower(modifier);
    }
    public void ShooterOff(){
        shooterMotorsRunning = false;
        SetShooterPower(0);
    }

    public void ShootThree(){
        ShooterOn(); //Spin up shooter

        stopShooterOverride = false;

        if(!shootRoutineRunning)shooterStartTime = currentOpmode.getRuntime();
        shootRoutineRunning = true;

        //wait
        while (shooterStartTime+timeInitialSpinup > currentOpmode.getRuntime()){
            currentOpmode.telemetry.addLine("waiting to spin up...");

            if(stopShooterOverride) { //if shooter should be stopped
                StopShooter();
                return;
            }
        }

        //Shoot three times
        ShootOne();
        ShooterOn();

        ShootOne();
        ShooterOn();

        ShootOne();

        StopShooter(); //Stop at end
        shootRoutineRunning = false;
    }
    public void ShootOne(){
        stopShooterOverride = false;

        if(!shootRoutineRunning)shooterStartTime = currentOpmode.getRuntime();
        shootRoutineRunning = true;

        //start shooter
        ShooterOn();

        //move starpath to storage position if needed
        if(starpathPosition < 3) StarpathToStorage();

        //wait
        while (shooterStartTime+timeToShoot > currentOpmode.getRuntime()){
            currentOpmode.telemetry.addLine("waiting to shoot...");

            if(stopShooterOverride) { //if shooter should be stopped
                StopShooter();
                return;
            }
        }

        //fire
        RotateStarpathToNextPos();

        //Stop Shooter
        StopShooter();
    }
    public void StopShooter(){
        shootRoutineRunning = false;
        ShooterOff();
    }

    public void ToggleShooterMotors(){ //turns shooter on or off
        if(shooterMotorsRunning) ShooterOff();
        else ShooterOn();
    }

    //Starpath
    public void StarpathToIntake(){
        starpathServo.setPosition(starpathIntakePosition);
        starpathPosition = 0;
    }
    public void StarPathToShooter(){
        starpathServo.setPosition(starpathShooterPosition);
        starpathPosition = 4;
    }
    public void StarpathToStorage(){
        starpathServo.setPosition(starpathStoragePosition);
        starpathPosition = 3;
    }
    public void RotateStarpathToNextPos(){
        // STARPATH POSITIONS //
        // 0, 1, 2 are intake positions
        // 3 is storage position
        // 4, 5, 6 are shooting positions

        starpathPosition++;

        //if next pos is 7, reset back to 0
        if(starpathPosition == 7) StarpathToIntake();
        //if next pos is 4, go to the shooter
        else if(starpathPosition == 4) StarPathToShooter();
        else if(starpathPosition == 3) StarpathToStorage();
        //else, add with the interval
        else if(starpathPosition < 4) StarpathAddIntervalIntake();
        else StarpathAddIntervalShooter();
    }
    public void RotateStarpathToPreviousPos(){
        if(starpathPosition != 0) starpathPosition--;

        if(starpathPosition == 3) starpathServo.setPosition(starpathIntakePosition + starpathIntervalIntake *3); //if the storage position
        else if (starpathPosition == 0) StarpathToIntake(); //if going to intake
        else if (starpathPosition == 4) StarPathToShooter(); // if going to shooter
        else if(starpathPosition == 2) starpathServo.setPosition(starpathIntakePosition + starpathIntervalIntake * 2); //if going to last intake pos
        else if (starpathPosition == 3) StarpathToStorage(); //if going to storage
        else if(starpathPosition > 4) StarpathSubtractIntervalShooter(); //if going to shooter position
        else StarpathSubtractIntervalIntake(); //if at an intake position
    }

    //Intake
    public void IntakeOn(){
        if(starpathPosition == 6) StarpathToIntake();
        if(starpathPosition >= 3) {//if in storage or shoot position, don't intake to prevent jams
            IntakeOff();
            return;
        }

        intakeRunning = true;
        intakeReverse = false;

        //Run intake servos
        intakeServo1.setPosition(1);
        intakeServo2.setPosition(0);
    }
    public void IntakeLoop(){
        if(!intakeRunning) return;


        //Manages auto-intaking
        boolean discInIntake = intakeDetector.getDistance(DistanceUnit.CM) < autoIntakeDistanceCM;
        boolean cooldownDone = lastAutoIntakeTime + autoIntakeCooldown < currentOpmode.getRuntime();
        if(starpathPosition < 3 && autoIntakeEnabled && discInIntake && cooldownDone){ //if hasn't previously detected a disc, move starpath
            RotateStarpathToNextPos();
            lastAutoIntakeTime = currentOpmode.getRuntime();
            /*//Stop intake servos
            intakeServo1.setPosition(0.5);
            intakeServo2.setPosition(0.5);*/
        }
        /*else if(cooldownDone && !intakeReverse){
            //Run intake servos
            intakeServo1.setPosition(1);
            intakeServo2.setPosition(0);
        }*/

        if(starpathPosition >= 3) {//if in storage or shoot position, don't intake to prevent jams
            IntakeOff();
            return;
        }
    }
    public void IntakeOff(){
        intakeRunning = false;
        intakeReverse = false;

        //Stop intake servos
        intakeServo1.setPosition(0.5);
        intakeServo2.setPosition(0.5);
    }
    public void ReverseIntake(){
        intakeReverse = true;
        intakeServo1.setPosition(0);
        intakeServo2.setPosition(1);
    }

    public void ToggleIntaking(){
        if(intakeRunning) IntakeOff();
        else IntakeOn();
    }

    ////Private Functions////
    //Shooter
    private void SetShooterPower(double power){
        int i = 0;
        for (DcMotor motor: shooterMotors) {
            motor.setPower(power);
            i++;
        }
    }

    //Starpath
    private void StarpathAddIntervalIntake(){starpathServo.setPosition(starpathServo.getPosition()+ starpathIntervalIntake);}
    private void StarpathSubtractIntervalIntake(){starpathServo.setPosition(starpathServo.getPosition()- starpathIntervalIntake);}
    private void StarpathAddIntervalShooter(){starpathServo.setPosition(starpathServo.getPosition()+ starpathIntervalShooter);}
    private void StarpathSubtractIntervalShooter(){starpathServo.setPosition(starpathServo.getPosition()- starpathIntervalShooter);}



}
