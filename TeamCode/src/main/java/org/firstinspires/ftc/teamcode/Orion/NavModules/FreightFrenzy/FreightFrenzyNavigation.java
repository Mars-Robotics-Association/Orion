package org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private DuckSpinner duckSpinner;
    private DistanceSensor duckDistance, intakeDistance;
    private DistanceSensor portTouch, starboardTouch;
    private ColorSensor colorSensor;
    private MecanumChassis chassis;
    private Camera camera;

    ////configuration variables////
    //Basic
    protected double whiteThreshold = 0.8;
    protected double wallStopDistance = 5;
    protected double timePastLineToWarehouse = 0.5;
    protected double timePastLineToHub = 0.5;

    //Ducks
    protected double duckStopDistance = 36;

    //Park Depot
    protected double depotParkAngle = 10;
    protected double depotStopDistance = 14;
    protected double depotParkTime = 1.8;

    //Park Warehouse
    protected double parkFurtherInWarehouseTime = 0.5;
    protected double parkFurtherInWarehouseAngle = 120;

    //Scan Barcode

    //Place

    //Collect

    ////configuration enums////
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

    boolean navigatorRunning = true;

    int currentNumberOfSpinCycles = 1;
    double currentParkFurtherInTime = 2;
    boolean currentStartAtDucks = false;
    boolean currentParkFurtherInWarehouse = false;
    double currentRobotSpeed = 0.5;

    public double sideMultiplier = 1;

    Thread thread;
    boolean threadRunning = false;

    public FreightFrenzyNavigation(OpMode setOpMode, UniversalTurretIntakeArm setArm, DuckSpinner setSpinner, DistanceSensor setDuckDist, DistanceSensor setIntakeDist, DistanceSensor setPortDist, DistanceSensor setStarboardDist, ColorSensor setColorSensor, AllianceSide setSide){
        opMode = setOpMode;
        arm = setArm;
        duckSpinner = setSpinner;
        duckDistance = setDuckDist;
        intakeDistance = setIntakeDist;
        portTouch = setPortDist;
        starboardTouch = setStarboardDist;
        colorSensor = setColorSensor;

        side = setSide;

        if(side == AllianceSide.RED) sideMultiplier = 1;
        else sideMultiplier = -1;
    }

    ////THREAD CODE////

    public void SetThread(Thread setThread) {thread=setThread;}

    //Runs in seperate thread
    @Override
    public void run() {
        opMode.telemetry.addLine("Nav Thread Start!");
        threadRunning = true;

        if(startSpinDucks) SpinDucksLinear(currentNumberOfSpinCycles,currentRobotSpeed);
        if(startParkWarehouse) ParkInWarehouseLinear(currentRobotSpeed,currentParkFurtherInWarehouse);
        if(startParkDepot) ParkInDepotLinear(currentStartAtDucks,currentRobotSpeed);
        if(startScanBarcode) ScanBarcodeLinear();
        if(startPlaceFreight) PlaceFreightLinear();
        if(startCollectFreight) CollectFreightLinear();

        opMode.telemetry.addLine("Nav Thread End!");

        startSpinDucks = false;
        startParkWarehouse = false;
        startParkDepot = false;
        startScanBarcode = false;
        startPlaceFreight = false;
        startCollectFreight = false;

        threadRunning = false;
    }
    public boolean IsThreadRunning(){return threadRunning;}


    ////MAJOR FUNCTIONS////
    public void StopNavigator(){navigatorRunning = false;}
    public void StartNavigator(){
        navigatorRunning = true;
        thread.start();
    }

    public void StartSpinDucks(){
        startSpinDucks = true;
        thread.start();
    }
    public void StartParkWarehouse(){
        startParkWarehouse = true;
        thread.start();
    }
    public void StartParkDepot(){
        startParkDepot = true;
        thread.start();
    }
    public void StartScanBarcode(){
        startScanBarcode = true;
        thread.start();
    }
    public void StartPlaceFreight(){
        startPlaceFreight = true;
        thread.start();
    }
    public void StartCollectFreight(){
        startSpinDucks = true;
        thread.start();
    }

    public void SpinDucksLinear(int numberOfCycles, double speed){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already (might need to turn? don't worry about it for now)
        GoToWall(90*sideMultiplier,speed);
        //wallFollow() until wallDist is close to north wall and wheel has contact with duck spinner
        WallFollowForDuckDistance(speed,duckStopDistance);
        //rampSpinDuck() for numberOfCycles
        for(int i = 0;i<numberOfCycles;i++) {
            //press against duck carousel
            DriveForTime(0, 0.5, 0.02*sideMultiplier, 0.2);
            //spin depending on side
            if(side == AllianceSide.BLUE) duckSpinner.GradSpin(true,0.5,1,opMode);
            else duckSpinner.GradSpin(false,0.5,1,opMode);
        }
        //Stop
        duckSpinner.Stop();
        chassis.Stop();
    }


    public void ParkInWarehouseLinear(double speed, boolean parkFurtherIn){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already
        GoToWall(90*sideMultiplier,speed);
        //wallFollow() until totally past white line
        WallFollowToWhite(-speed);
        WallFollowForTime(-speed,timePastLineToWarehouse);
        //go towards the center a bit if parkFurtherIn
        if(parkFurtherIn) DriveForTime(parkFurtherInWarehouseAngle,speed,0, parkFurtherInWarehouseTime);
        //stop
        chassis.Stop();
    }

    public void ParkInDepotLinear(boolean startsAtDucks, double speed){
        //should start along side wall north of warehouse with intake facing south
        //if startsAtDucks, skip next two steps
        if(!startsAtDucks) {
            //goToWall() if not already at it
            GoToWall(90*sideMultiplier, speed);
            //wallFollow() to ducks on north wall
            WallFollowForDuckDistance(speed,duckStopDistance);
        }
        //Go diagonal back towards middle of field for a time
        //Go north until against the north wall

        duckSpinner.Stop();

        //move to park
        DriveForTime(90+(sideMultiplier*depotParkAngle),sideMultiplier*0.5,0,depotParkTime);

        DriveForDuckSensorDistance(0,0.5,0,depotStopDistance);
        chassis.RawDrive(0,0,0);

        /*if(side == FreightFrenzyNavigation.AllianceSide.RED){
            while (!(colorSensor.red() >= 255-thresh && colorSensor.green() <= thresh && colorSensor.blue() <= thresh)) {
                chassis.RawDrive(depotAngle,speed,0);
            }
        }else{
            while (!(colorSensor.blue() >= 255-thresh && colorSensor.green() <= thresh && colorSensor.red() <= thresh)) {
                chassis.RawDrive(depotAngle,speed,0);
            }
        }
        chassis.RawDrive(0,0,0);*/
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
        if(camera.countPixels(first)>camera.countPixels(second)&&camera.countPixels(first)>camera.countPixels(third)){
            pos=DuckPos.FIRST;
            opMode.telemetry.addData("Element in position","1");
        }
        if(camera.countPixels(second)>camera.countPixels(first)&&camera.countPixels(second)>camera.countPixels(third)){
            pos=DuckPos.SECOND;
            opMode.telemetry.addData("Element in position","2");
        }
        if(camera.countPixels(third)>camera.countPixels(second)&&camera.countPixels(third)>camera.countPixels(first)){
            pos=DuckPos.THIRD;
            opMode.telemetry.addData("Element in position","3");
        }
        opMode.telemetry.update();
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

    //Drive until duck sensor detects distance
    public void DriveForDuckSensorDistance(double angle, double speed, double turnOffset, double distance){
        while (duckDistance.getDistance(DistanceUnit.CM)>distance){
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
        while (portTouch.getDistance(DistanceUnit.CM) < 6 || starboardTouch.getDistance(DistanceUnit.CM)<6) {
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

    //Wall follows at specified speed, which also determines direction .
    public void WallFollowForDuckDistance(double speed, double distance){
        double turnOffset = 0;
        if(side == AllianceSide.BLUE) turnOffset = -0.04*speed;
        if(side == AllianceSide.RED) turnOffset = 0.04*speed;

        while (duckDistance.getDistance(DistanceUnit.CM)>distance){
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
