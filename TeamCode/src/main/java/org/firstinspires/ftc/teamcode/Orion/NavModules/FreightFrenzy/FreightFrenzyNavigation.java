package org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.NavModules.Camera;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.DuckSpinner;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

@Config
public class FreightFrenzyNavigation implements Runnable
{
    //relevant sensors and controllers
    private OpMode opMode;
    private UniversalTurretIntakeArm arm;
    private DuckSpinner duckSpinner;
    private DistanceSensor duckDistance, intakeDistance;
    private DistanceSensor portDist, starboardDist;
    private ColorSensor colorSensor;
    private MecanumChassis chassis;
    private Camera camera;

    ////configuration variables////
    //Basic
    protected double whiteThreshold = 100;
    protected double wallStopDistance = 14;
    protected double timePastLineToWarehouse = 0.5;
    protected double timePastLineToHub = 0.5;
    public static double turnCoefficient = 0.02;


    //Ducks
    protected double duckStopDistance = 34;

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
    public enum DuckPos {FIRST,SECOND,THIRD,NULL}
    public AllianceSide side = AllianceSide.BLUE;
    public void ToggleAllianceSide(){
        if(side == AllianceSide.BLUE)  side=AllianceSide.RED;
        else side=AllianceSide.BLUE;

        if(side == AllianceSide.RED) sideMultiplier = 1;
        else sideMultiplier = -1;
    }
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

    protected double sideMultiplier = 1;
    public double GetSideMultiplier(){
        if(side == AllianceSide.RED) sideMultiplier = 1;
        else sideMultiplier = -1;
        return sideMultiplier;
    }

    Thread thread;
    boolean threadRunning = false;

    public FreightFrenzyNavigation(OpMode setOpMode, MecanumChassis setChassis, UniversalTurretIntakeArm setArm, DuckSpinner setSpinner, DistanceSensor setDuckDist, DistanceSensor setIntakeDist, DistanceSensor setPortDist, DistanceSensor setStarboardDist, ColorSensor setColorSensor, AllianceSide setSide){
        opMode = setOpMode;
        chassis=setChassis;
        arm = setArm;
        duckSpinner = setSpinner;
        duckDistance = setDuckDist;
        intakeDistance = setIntakeDist;
        portDist = setPortDist;
        starboardDist = setStarboardDist;
        colorSensor = setColorSensor;
        camera = new Camera(opMode,"Webcam 1");

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
        NavigatorOn();

        if(startSpinDucks) {
            startSpinDucks = false;
            DriveAndSpinDucksLinear(currentNumberOfSpinCycles,currentRobotSpeed);
        }
        if(startParkWarehouse) {
            startParkWarehouse = false;
            ParkInWarehouseLinear(currentRobotSpeed,currentParkFurtherInWarehouse);
        }
        if(startParkDepot) {
            startParkDepot = false;
            ParkInDepotLinear(currentStartAtDucks,currentRobotSpeed);
        }
        if(startScanBarcode) {
            startScanBarcode = false;
            ScanBarcodeLinear();
        }
        if(startPlaceFreight) {
            startPlaceFreight = false;
            PlaceFreightLinear();
        }
        if(startCollectFreight) {
            startCollectFreight = false;
            CollectFreightLinear();
        }

        opMode.telemetry.addLine("Nav Thread End!");


        threadRunning = false;
    }
    public boolean IsThreadRunning(){return threadRunning;}


    ////MAJOR FUNCTIONS////
    public void StopNavigator(){navigatorRunning = false;}
    public void NavigatorOn(){
        navigatorRunning = true;
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
        startCollectFreight = true;
        thread.start();
    }

    public void DriveAndSpinDucksLinear(int numberOfCycles, double speed){
        NavigatorOn();
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already (might need to turn? don't worry about it for now)
        GoToWall(speed);
        //wallFollow() until wallDist is close to north wall and wheel has contact with duck spinner
        WallFollowForDuckDistance(speed,duckStopDistance);
        //rampSpinDuck() for numberOfCycles
        for(int i = 0;i<numberOfCycles;i++) {
            SpinDucks(0.5,1);
        }
        //Stop
        duckSpinner.Stop();
        chassis.RawDrive(0,0,0);
    }


    public void ParkInWarehouseLinear(double speed, boolean parkFurtherIn){
        //should start along side wall north of warehouse with intake facing south
        //goToWall() if not at it already
        GoToWall(90*sideMultiplier,speed);
        //wallFollow() until totally past white line
        WallFollowToWhite(-speed,0);
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

    public DuckPos ScanBarcodeLinear(){
        //should start at constant point along a side wall north of warehouse with intake facing south
        //start moving arm so intake faces barcode
        //break away from the wall and move towards the center of the field for time
        //sweep along the possible points towards the shipping hub (could use tensorflow/openCV instead if works reliably)
        //use intakeDist to record the time from start of sweep at which it detected the freight
        //determine barcode configuration off of time
        //raise arm to appropriate height and reverse intake
        DriveForTime(0,1,0,1);
        double startTime = opMode.getRuntime();
        DriveForDuckSensorDistance(90,0.5,0,5);
        double totalTime = opMode.getRuntime()-startTime;
        DuckPos location = DuckPos.NULL;
        if(totalTime<2){
            location = DuckPos.FIRST;
        }else if(totalTime<3){
            location = DuckPos.SECOND;
        }else {
            location = DuckPos.THIRD;
        }
        return location;
    }

    public DuckPos ScanBarcodeOpenCV() throws InterruptedException {
        //get camera input and convert to mat
        //divide image into three sections
        //find section with most yellow
        DuckPos pos= DuckPos.NULL;
        Bitmap in = camera.GetImage();
        Mat img = camera.convertBitmapToMat(in);
        Rect firstRect = new Rect(0,0,img.width()/3,img.height());
        Rect secondRect = new Rect(img.width()/3,0,img.width()/3,img.height());
        Rect thirdRect = new Rect(2*img.width()/3,0,img.width()/3,img.height());
        Mat firstMat = new Mat(img,firstRect);
        Mat secondMat = new Mat(img,secondRect);
        Mat thirdMat = new Mat(img,thirdRect);
        FtcDashboard.getInstance().sendImage(camera.convertMatToBitMap(firstMat));
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
        else if(camera.countPixels(second)>camera.countPixels(first)&&camera.countPixels(second)>camera.countPixels(third)){
            pos=DuckPos.SECOND;
            opMode.telemetry.addData("Element in position","2");
        }
        else if(camera.countPixels(third)>camera.countPixels(second)&&camera.countPixels(third)>camera.countPixels(first)){
            pos=DuckPos.THIRD;
            opMode.telemetry.addData("Element in position","3");
        }
        if(pos==DuckPos.NULL) {
            opMode.telemetry.addData("Element in position", "null");
        }
        opMode.telemetry.update();
        return DuckPos.FIRST;
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

        TurnToAngle(0,0.8);
        TurnToAngle(0,0.2);
        GoToWall(1);
        WallFollowToWhite(0.6,0);
        WallFollowForTime(1,0.25);
        DriveForTime(45*sideMultiplier,1,0,0.65);
        TurnToAngle(75*sideMultiplier,0.5);
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

        TurnToAngle(0,0.4);
        GoToWall(1);
        arm.ReturnToHomeAndIntake(0.02,1);
        WallFollowToWhite(0.6,180);
    }

    ////MINOR FUNCTIONS////

    //Drive for a period of time
    public void DriveForTime(double angle, double speed, double turnOffset, double time){
        double startTime = opMode.getRuntime();
        while (opMode.getRuntime()<startTime+time && navigatorRunning){
            chassis.RawDrive(angle,speed,turnOffset);
        }
        chassis.RawDrive(0,0,0);
    }

    //Drive until duck sensor detects distance
    public void DriveForDuckSensorDistance(double angle, double speed, double turnOffset, double distance){
        while (duckDistance.getDistance(DistanceUnit.CM)>distance && navigatorRunning){
            chassis.RawDrive(angle,speed,turnOffset);
        }
        chassis.RawDrive(0,0,0);
    }

    //Wait for a period of time
    public void Wait(double time){
        double startTime = opMode.getRuntime();
        while (opMode.getRuntime()<startTime+time && navigatorRunning){}

    }

    //Goes to the wall at an angle. Stops when in contact with wall
    public void GoToWall(double angle, double speed){
        while (!CheckDistance(portDist,wallStopDistance) && !CheckDistance(starboardDist,wallStopDistance) && navigatorRunning) {
            chassis.RawDrive(angle, speed, 0);
        }
        chassis.RawDrive(0,0,0);
    }
    public void GoToWall(double speed){
        GoToWall(90*(-sideMultiplier),speed);
    }

    //Wall follows at specified speed, which also determines direction.
    public void WallFollowForTime(double speed, double time){
        double turnOffset = 0;
        if(side == AllianceSide.BLUE) turnOffset = -0.02*speed;
        if(side == AllianceSide.RED) turnOffset = 0.02*speed;

        double startTime = opMode.getRuntime();
        while (opMode.getRuntime()<startTime+time && navigatorRunning){
            chassis.RawDrive(0,speed,turnOffset);
        }
        chassis.RawDrive(0,0,0);
    }

    //Wall follows at specified speed, which also determines direction .
    public void WallFollowForDuckDistance(double speed, double distance){
        double turnOffset = 0;
        if(side == AllianceSide.BLUE) turnOffset = -0.02*speed;
        if(side == AllianceSide.RED) turnOffset = 0.02*speed;

        while (!CheckDistance(duckDistance,distance) && navigatorRunning){
            chassis.RawDrive(0,speed,turnOffset);
        }
        chassis.RawDrive(0,0,0);
    }

    //Wall follows until white is detected by colorSensor. Must be called every loop().
    public void WallFollowToWhite(double speed, double angle){
        double turnOffset = 0.02*speed*sideMultiplier;
        if(angle>0) turnOffset*=-1;

        while (colorSensor.alpha() < whiteThreshold && navigatorRunning){
            chassis.RawDrive(angle,speed,turnOffset);
        }
        chassis.RawDrive(0,0,0);

    }

    //Turns to an angle
    public void TurnToAngle(double angle, double speed){
        while (!chassis.InWithinRangeOfAngle(angle,5) && navigatorRunning) {
            chassis.TurnTowardsAngle(angle, speed, turnCoefficient);
            opMode.telemetry.addData("Robot Angle", chassis.GetImu().GetRobotAngle());
            opMode.telemetry.update();
        }
    }

    //Spine the ducks
    public void SpinDucks(double multiplier, double maxSpeed){
        NavigatorOn();
        //DriveForTime(-90,0.5,0.08*sideMultiplier,0.2);
        double startTime = opMode.getRuntime();
        double speed = 0;
        while(speed<maxSpeed && navigatorRunning){

            speed = (opMode.getRuntime()-startTime)*multiplier;
            if (speed>1)speed=1;
            if(speed>maxSpeed)speed = maxSpeed;
            duckSpinner.SetSpeed(speed*(-sideMultiplier));

            chassis.RawDrive(-75*sideMultiplier, 0.3, 0.04*sideMultiplier);
        }
        duckSpinner.SetSpeed(0);
    }

    //Uses the webcam to zero in on specified tfObject. Takes into account the bearing of the arm/webcam.
    public void ZeroIn(int objectID, double armRotation){

    }

    public void PrintSensorTelemetry(){
        Telemetry tele = opMode.telemetry;
        //tele.addData("Duck Distance", duckDistance.getDistance(DistanceUnit.CM));
        //tele.addData("Intake Distance", intakeDistance.getDistance(DistanceUnit.CM));
        //tele.addData("Port Distance", portDist.getDistance(DistanceUnit.CM));
        //tele.addData("Starboard Distance", starboardDist.getDistance(DistanceUnit.CM));
        tele.addData("Color Alpha", colorSensor.alpha());
    }

    boolean CheckDistance(DistanceSensor sensor,double distance){
        double dist = sensor.getDistance(DistanceUnit.CM);
        if(!(dist>0)) return false;//if distance is not greater than 0
        if(dist<distance) return true;
        else return false;
    }
}
