package org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.NavModules.Camera;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.DuckSpinner;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

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
    private Camera camera;

    //configuration variables
    protected double whiteThreshold = 0.8;

    //configuration enums
    public enum AllianceSide {RED, BLUE}
    public enum ShippingHubChoice {ALLIANCE, SHARED}
    public enum Tier {BOTTOM, MIDDLE, TOP}
    public enum DuckPos {FIRST,SECOND,THIRD};
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
    double currentParkFurtherInTime = 2;
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
        camera = new Camera(opMode,"Webcam 1");
    }

    ////THREAD CODE////

    @Override
    public void run() {
        if(startSpinDucks) SpinDucksLinear(currentNumberOfSpinCycles,20,1,5,0.1);
        if(startParkWarehouse) ParkInWarehouseLinear(currentParkFurtherInTime,20,1);
        if(startParkDepot) ParkInDepotLinear(currentStartAtDucks,20,20,1,2,10);
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


    ////MAJOR FUNCTIONS////

    public void SpinDucksLinear(int numberOfCycles, double angle, double speed, double time, double speedMultiplier){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already (might need to turn? don't worry about it for now)
        //wallFollow() until wallDist is close to north wall and wheel has contact with duck spinner
        //apply constant pressure towards the wall
        //rampSpinDuck() for numberOfCycles
        GoToWall(angle,speed);
        WallFollowForTime(speed,time);
        double start = opMode.getRuntime();
        for(int i = 0;i<numberOfCycles;i++) {
            if(side == AllianceSide.BLUE) {
                spinner.GradSpin(true,0.1,1,opMode);
            } else{
                spinner.GradSpin(false,0.1,1,opMode);
            }
        }
        spinner.Stop();
    }


    public void ParkInWarehouseLinear(double extraTime,double angle,double speed){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already
        //wallFollow() until totally past white line
        //go towards the center a bit if parkFurtherIn
        GoToWall(angle,speed);
        WallFollowToWhite(speed);
        DriveForTime(angle,speed,0,extraTime+1);
        chassis.RawDrive(0,0,0);
    }

    public void ParkInDepotLinear(boolean startsAtDucks, double followAngle, double depotAngle, double speed, double time, int thresh){
        //should start along side wall north of warehouse with intake facing south
        //if startsAtDucks, skip next two steps
        //goToWall() if not already at it
        //wallFollow() to ducks on north wall
        //Go diagonal back towards middle of field for a time
        //Go north until against the north wall
        if(!startsAtDucks) {
            GoToWall(followAngle, speed);
            WallFollowForTime(speed,time);
        }
        if(side == FreightFrenzyNavigation.AllianceSide.RED){
            while (!(colorSensor.red() >= 255-thresh && colorSensor.green() <= thresh && colorSensor.blue() <= thresh)) {
                chassis.RawDrive(depotAngle,speed,0);
            }
        }else{
            while (!(colorSensor.blue() >= 255-thresh && colorSensor.green() <= thresh && colorSensor.red() <= thresh)) {
                chassis.RawDrive(depotAngle,speed,0);
            }
        }
        chassis.RawDrive(0,0,0);
    }

    public void ScanBarcodeLinear(){
        //should start at constant point along a side wall north of warehouse with intake facing south
        //start moving arm so intake faces barcode
        //break away from the wall and move towards the center of the field for time
        //sweep along the possible points towards the shipping hub (could use tensorflow/openCV instead if works reliably)
        //use intakeDist to record the time from start of sweep at which it detected the freight
        //determine barcode configuration off of time
        //raise arm to appropriate height and reverse intake

    }

    public void ScanBarcodeOpenCV(){
        //get camera input and convert to mat
        //divide image into three sections
        //find section with most yellow
        DuckPos pos;
        Bitmap in = camera.GetImage();
        Mat img = camera.convertBitmapToMat(in);
        Rect firstRect = new Rect(0,0,img.width()/3,img.height());
        Rect secondRect = new Rect(img.width()/3,0,img.width()/3,img.height());
        Rect thirdRect = new Rect(2*img.width()/3,0,img.width()/3,img.height());
        Mat firstMat = new Mat(img,firstRect);
        Mat secondMat = new Mat(img,secondRect);
        Mat thirdMat = new Mat(img,thirdRect);
        firstMat = camera.IsolateYellow(firstMat);
        secondMat = camera.IsolateYellow(secondMat);
        thirdMat = camera.IsolateYellow(thirdMat);
        Bitmap first = camera.convertMatToBitMap(firstMat);
        Bitmap second = camera.convertMatToBitMap(secondMat);
        Bitmap third = camera.convertMatToBitMap(thirdMat);
        if(camera.countPixels(first)>camera.countPixels(second)&&camera.countPixels(first)>camera.countPixels(third)) pos=DuckPos.FIRST;
        if(camera.countPixels(second)>camera.countPixels(first)&&camera.countPixels(second)>camera.countPixels(third)) pos=DuckPos.SECOND;
        if(camera.countPixels(third)>camera.countPixels(second)&&camera.countPixels(third)>camera.countPixels(first)) pos=DuckPos.THIRD;
    }

    public void PlaceFreightLinear(){
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

    public void CollectFreightLinear(){
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
    public void ZeroIn(int objectID, double armRotation){

    }
}
