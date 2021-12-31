package org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.CuriosityRobot;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.DuckSpinner;

public class FreightFrenzyAutoMethods {
    //relevant sensors and controllers
    private UniversalTurretIntakeArm arm;
    private DuckSpinner spinner;
    private DistanceSensor duckDistance, intakeDistance;
    private TouchSensor portTouch, starboardTouch;
    private ColorSensor colorSensor;
    private MecanumChassis chassis;
    private OpMode opmode;

    //configuration variables

    //configuration enums
    public enum AllianceSide {RED, BLUE}
    public enum ShippingHubChoice {ALLIANCE, SHARED}
    public enum Tier {BOTTOM, MIDDLE, TOP}
    FreightFrenzyAutoMethods.AllianceSide side = FreightFrenzyAutoMethods.AllianceSide.BLUE;
    FreightFrenzyAutoMethods.ShippingHubChoice currentHubChoice = FreightFrenzyAutoMethods.ShippingHubChoice.ALLIANCE;
    FreightFrenzyAutoMethods.Tier currentTier = FreightFrenzyAutoMethods.Tier.MIDDLE;

    public FreightFrenzyAutoMethods(OpMode setOpMode, FreightFrenzyNavTuningProfile profile, AllianceSide color, MecanumChassis setChassis){
        arm = profile.arm();
        spinner = profile.spinner();
        duckDistance = profile.duckDistance();
        intakeDistance = profile.intakeDistance();
        portTouch = profile.portTouch();
        starboardTouch = profile.starboardTouch();
        colorSensor = profile.colorSensor();
        chassis = setChassis;
        side = color;
        opmode = setOpMode;
    }

    ////MAJOR FUNCTIONS////

    public void SpinDucks(int numberOfCycles){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already (might need to turn? don't worry about it for now)
        //wallFollow() until wallDist is close to north wall and wheel has contact with duck spinner
        //apply constant pressure towards the wall
        //rampSpinDuck() for numberOfCycles
        GoToWall(45,1);
        WallFollow(1);
        double start = opmode.getRuntime();
        if(side == AllianceSide.BLUE) {
            spinner.GradSpin(true,0.1,1,opmode);
        }
        else{
            spinner.GradSpin(false,0.1,1,opmode);
        }
        while(opmode.getRuntime()<start+(3*numberOfCycles)){
            opmode.telemetry.addData("waiting","");
        }
        spinner.Stop();
    }

    public void ParkInWarehouse(double parkFurtherIn){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already
        //wallFollow() until totally past white line
        //go towards the center a bit if parkFurtherIn
        if(side == AllianceSide.BLUE) {
            GoToWall(45,1);
            WallFollowToWhite(-1);
            double oldrun = opmode.getRuntime();
            while(opmode.getRuntime()<oldrun+parkFurtherIn)
            {
                 chassis.RawDrive(0,-1,0);
            }
            chassis.RawDrive(0,0,0);
        }else{

            GoToWall(-45,1);
            WallFollowToWhite(-1);
            double oldrun = opmode.getRuntime();
            while(opmode.getRuntime()<oldrun+parkFurtherIn)
            {
                chassis.RawDrive(0,-1,0);
            }
            chassis.RawDrive(0,0,0);
        }
    }

    public void ParkInDepot(boolean startsAtDucks){
        //should start along side wall north of warehouse with intake facing south
        //if startsAtDucks, skip next two steps
        //goToWall() if not already at it
        //wallFollow() to ducks on north wall
        //Go diagonal back towards middle of field for a time
        //Go north until against the north wall
        if(side==AllianceSide.BLUE) {
            if(!startsAtDucks) {
                GoToWall(45, 1);
                WallFollow(1);
            }
            while (!(colorSensor.red() >= 240 && colorSensor.green() <= 10 && colorSensor.blue() <= 10)) {
                chassis.RawDrive(100,0.5,0);
            }
            chassis.RawDrive(0,0,0);
        }else{
            if(!startsAtDucks) {
                GoToWall(-45, 1);
                WallFollow(1);
            }
            while (!(colorSensor.red() >= 240 && colorSensor.green() <= 10 && colorSensor.blue() <= 10)) {
                chassis.RawDrive(80,-0.5,0);
            }
            chassis.RawDrive(0,0,0);
        }
    }

    public void ScanBarcode(){
        //should start at constant point along a side wall north of warehouse with intake facing south
        //start moving arm so intake faces barcode
        //break away from the wall and move towards the center of the field for time
        //sweep along the possible points towards the shipping hub (could use tensorflow/openCV instead if works reliably)
        //use intakeDist to record the time from start of sweep at which it detected the freight
        //determine barcode configuration off of time
        //raise arm to appropriate height and reverse intake
    }

    public void PlaceFreight(){
        //should start in the depot with the intake side facing away from hub (can start on white line)
        //goToWall()
        //wallFollowToWhite() from the south
        //rotate arm to face shipping hub
        //raise arm to the correct height
        //dead reckon for time towards shipping hub's location
        //zeroIn() on shipping hub using tensorflow
        //when close enough, reverse intake
        //when no freight is detected by intakeDist, goToWall() at a diagonal
        //wallFollowToWhite() from the north
    }

    public void CollectFreight(){
        //should start north of white line along wall or at it
        //goToWall()
        //wallFollowToWhite() from the north
        //wallFollow() into the depot
        //select a freight to zeroIn() on
        //autoIntake() while zeroIn()-ing on freight
        //once freight is collected, goToWall() at a diagonal
        //wallFollowToWhite() from the south
    }

    ////MINOR FUNCTIONS////

    //Goes to the wall at an angle. Stops when in contact with wall
    public void GoToWall(double angle, double speed){

    }

    //Wall follows at specified speed, which also determines direction.
    public void WallFollow(double speed){

    }

    //Wall follows until white is detected by colorSensor. Must be called every loop().
    public void WallFollowToWhite(double speed){

    }

    //Uses the webcam to zero in on specified tfObject. Takes into account the bearing of the arm/webcam.
    public void ZeroIn(int objectID){

    }
}
