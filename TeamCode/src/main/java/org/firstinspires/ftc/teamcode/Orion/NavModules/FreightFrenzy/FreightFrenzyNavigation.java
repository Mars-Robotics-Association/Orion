package org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.DuckSpinner;

public class FreightFrenzyNavigation implements Runnable
{
    //relevant sensors and controllers
    private OpMode opMode;
    private UniversalTurretIntakeArm arm;
    private DuckSpinner spinner;
    private DistanceSensor duckDistance, intakeDistance;
    private TouchSensor portTouch, starboardTouch;
    private ColorSensor colorSensor;
    private MecanumChassis chassis;

    //configuration variables
    protected double whiteThreshold = 0.8;

    //configuration enums
    public enum AllianceSide {RED, BLUE}
    public enum ShippingHubChoice {ALLIANCE, SHARED}
    public enum Tier {BOTTOM, MIDDLE, TOP}
    AllianceSide side = AllianceSide.BLUE;
    ShippingHubChoice currentHubChoice = ShippingHubChoice.ALLIANCE;
    Tier currentTier = Tier.MIDDLE;

    //function running in thread variables
    boolean startSpinDucks = false;
    boolean startParkWarehouse = false;
    boolean startParkDepot = false;
    boolean startScanBarcode = false;
    boolean startPlaceFreight = false;
    boolean startCollectFreight = false;

    int currentNumberOfSpinCycles = 1;
    boolean currentParkFurtherIn = false;
    boolean currentStartAtDucks = false;

    public FreightFrenzyNavigation(OpMode setOpMode, FreightFrenzyNavTuningProfile profile){
        opMode = setOpMode;
        arm = profile.arm();
        spinner = profile.spinner();
        duckDistance = profile.duckDistance();
        intakeDistance = profile.intakeDistance();
        portTouch = profile.portTouch();
        starboardTouch = profile.starboardTouch();
        colorSensor = profile.colorSensor();
    }

    ////THREAD CODE////

    @Override
    public void run() {
        if(startSpinDucks) SpinDucksLinear(currentNumberOfSpinCycles);
        if(startParkWarehouse) ParkInWarehouseLinear(currentParkFurtherIn);
        if(startParkDepot) ParkInDepotLinear(currentStartAtDucks);
        if(startScanBarcode) ScanBarcodeLinear();
        if(startPlaceFreight) PlaceFreightLinear();
        if(startCollectFreight) CollectFreightLinear();

        startSpinDucks = false;
        startParkWarehouse = false;
        startParkDepot = false;
        startScanBarcode = false;
        startPlaceFreight = false;
        startCollectFreight = false;
    }

    ////MAJOR CALLABLE FUNCTIONS////

    public void SpinDucks(int numberOfCycles){
        currentNumberOfSpinCycles = numberOfCycles;
        startSpinDucks = true;
    }
    public void ParkInWarehouse(boolean parkFurtherIn){
        currentParkFurtherIn = parkFurtherIn;
        startParkWarehouse = true;
    }
    public void ParkInDepot(boolean startAtDucks){
        currentStartAtDucks = startAtDucks;
        startParkDepot = true;
    }
    public void ScanBarcode(){startScanBarcode = true;}
    public void PlaceFreight(){startPlaceFreight = true;}
    public void CollectFreight(){startCollectFreight = true;}


    ////MAJOR INTERNAL FUNCTIONS////

    private void SpinDucksLinear(int numberOfCycles){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already (might need to turn? don't worry about it for now)
        //wallFollow() until wallDist is close to north wall and wheel has contact with duck spinner
        //apply constant pressure towards the wall
        //rampSpinDuck() for numberOfCycles
    }

    private void ParkInWarehouseLinear(boolean parkFurtherIn){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already
        //wallFollow() until totally past white line
        //go towards the center a bit if parkFurtherIn
    }

    private void ParkInDepotLinear(boolean startsAtDucks){
        //should start along side wall north of warehouse with intake facing south
        //if startsAtDucks, skip next two steps
        //goToWall() if not already at it
        //wallFollow() to ducks on north wall
        //Go diagonal back towards middle of field for a time
        //Go north until against the north wall
    }

    private void ScanBarcodeLinear(){
        //should start at constant point along a side wall north of warehouse with intake facing south
        //start moving arm so intake faces barcode
        //break away from the wall and move towards the center of the field for time
        //sweep along the possible points towards the shipping hub (could use tensorflow/openCV instead if works reliably)
        //use intakeDist to record the time from start of sweep at which it detected the freight
        //determine barcode configuration off of time
        //raise arm to appropriate height and reverse intake
    }

    private void PlaceFreightLinear(){
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

    private void CollectFreightLinear(){
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

    //Drive for a period of time
    public void DriveForTime(double angle, double speed, double turnOffset, double time){
        double startTime = opMode.getRuntime();
        while (opMode.getRuntime()<startTime+time){
            chassis.RawDrive(angle,speed,turnOffset);
        }
        chassis.RawDrive(0,0,0);
    }

    //Wait for a period of time
    public void Wait(double time){
        double startTime = opMode.getRuntime();
        while (opMode.getRuntime()<startTime+time){}

    }

    //Goes to the wall at an angle. Stops when in contact with wall
    public void GoToWall(double angle, double speed){
        while (portTouch.isPressed() || starboardTouch.isPressed()) {
            chassis.RawDrive(angle, speed, 0);
        }
        chassis.RawDrive(0,0,0);
    }

    //Wall follows at specified speed, which also determines direction.
    public void WallFollowForTime(double speed, double time){
        double turnOffset = 0;
        if(side == AllianceSide.BLUE) turnOffset = -0.04*speed;
        if(side == AllianceSide.RED) turnOffset = 0.04*speed;

        double startTime = opMode.getRuntime();
        while (opMode.getRuntime()<startTime+time){
            chassis.RawDrive(0,speed,turnOffset);
        }
        chassis.RawDrive(0,0,0);
    }

    //Wall follows until white is detected by colorSensor. Must be called every loop().
    public void WallFollowToWhite(double speed){
        double turnOffset = 0;
        if(side == AllianceSide.BLUE) turnOffset = -0.04*speed;
        if(side == AllianceSide.RED) turnOffset = 0.04*speed;

        while (colorSensor.alpha() < whiteThreshold){
            chassis.RawDrive(0,speed,turnOffset);
        }
        chassis.RawDrive(0,0,0);

    }

    //Uses the webcam to zero in on specified tfObject. Takes into account the bearing of the arm/webcam.
    public void ZeroIn(int objectID){

    }
}
