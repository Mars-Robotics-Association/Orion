package org.firstinspires.ftc.teamcode.Navigation.FreightFrenzy;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;
import org.firstinspires.ftc.teamcode._RobotCode.Erasmus.DuckSpinner;
import org.firstinspires.ftc.teamcode._RobotCode.Erasmus.ErasmusTurretArm;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Config
public class FreightFrenzyNavigator
{
   ////Dependencies////
   //Classes- higher (passes in through constructor)
   private OpMode opMode;
   private BaseRobot robot;
   private ErasmusTurretArm arm;
   private DuckSpinner duckSpinner;
   private MecanumChassis chassis;
   private BlinkinController lights;
   //Classes- lower (instantiated by this)
   private Camera camera;
   private ChassisFunctions driveFuncs;
   private FreightFrenzyNavigatorThread navigatorThread;

   ////Sensors////
   public DistanceSensor duckDistance, intakeDistance, levelSensor;
   public DistanceSensor portDist, starboardDist;
   public ColorSensor colorSensor;

   ////Configuration Variables////
   //Basic
   protected double timePastLineToWarehouse = 0.5;
   public static double turnWhileDrivingCoefficient = 0.015;

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
   protected double placeHeightThreshold = 30; //uses distance sensor on the bottom to know when to stop
   protected double placeTurningCoefficient = 0.05; //multiplier by error for turn offset
   protected double placeSpeed = 0.5;
   public static double goToPlaceFacingAngle = 120;
   public static double goToPlaceTime = 0.65;

   //Collect
   protected double currentArmAutoHeight = 0.02;
   public void SetArmAutoHeight(double armRaiseHeight) { currentArmAutoHeight = armRaiseHeight; }

   ////Configuration Enums////
   public enum DuckPos {FIRST,SECOND,THIRD,NULL}

   //side multiplier
   protected double sideMultiplier = 1;
   public double CalculateSideMultiplier(){
      if(robot.fieldSide == BaseRobot.FieldSide.RED) sideMultiplier = 1;
      else sideMultiplier = -1;
      return sideMultiplier;
   }

   private boolean navigatorRunning = true;
   public boolean IsNavigatorRunning(){return navigatorRunning;}
   public void StopNavigator(){navigatorRunning = false;}
   public void NavigatorOn(){
      navigatorRunning = true;
   }


   public FreightFrenzyNavigator(OpMode setOpMode, BaseRobot setRobot, MecanumChassis setChassis,
                                 ErasmusTurretArm setArm, DuckSpinner setSpinner,
                                 DistanceSensor setDuckDist, DistanceSensor setIntakeDist, DistanceSensor setLevelSensor,
                                 DistanceSensor setPortDist, DistanceSensor setStarboardDist, ColorSensor setColorSensor,
                                 BlinkinController setBlinkin) {
      //assign pass-through references
      opMode = setOpMode;
      robot =setRobot;
      chassis=setChassis;
      arm = setArm;
      lights = setBlinkin;
      duckSpinner = setSpinner;
      //assign sensors
      duckDistance = setDuckDist;
      intakeDistance = setIntakeDist;
      levelSensor = setLevelSensor;
      portDist = setPortDist;
      starboardDist = setStarboardDist;
      colorSensor = setColorSensor;
      //instantiate navigator-specific classes
      camera = new Camera(opMode,"Webcam 1");
      driveFuncs = new ChassisFunctions(opMode, chassis, this);
      navigatorThread = new FreightFrenzyNavigatorThread(opMode, lights, this);

      sideMultiplier = CalculateSideMultiplier();
   }



   public void DriveAndSpinDucksLinear(int numberOfCycles, double speed){
      NavigatorOn();
      //should start along side wall north of warehouse with intake facing south
      //goToWall() if not at it already (might need to turn? don't worry about it for now)
      driveFuncs.GoToWall(speed);
      //wallFollow() until wallDist is close to north wall and wheel has contact with duck spinner
      driveFuncs.WallFollowForDuckDistance(speed,duckStopDistance);
      //rampSpinDuck() for numberOfCycles
      for(int i = 0;i<numberOfCycles;i++) {
         if(!navigatorRunning) break;
         SpinDucks(0.5,1);
      }
      //Stop
      duckSpinner.Stop();
      chassis.RawDrive(0,0,0);
   }


   public void ParkInWarehouseLinear(double speed, boolean parkFurtherIn){
      //should start along side wall north of warehouse with intake facing south
      //goToWall() if not at it already
      driveFuncs.GoToWall(90*sideMultiplier,speed);
      //wallFollow() until totally past white line
      driveFuncs.WallFollowToWhite(-speed,0);
      driveFuncs.WallFollowForTime(-speed,timePastLineToWarehouse);
      //go towards the center a bit if parkFurtherIn
      if(parkFurtherIn) driveFuncs.DriveForTime(parkFurtherInWarehouseAngle,speed,0, parkFurtherInWarehouseTime);
      //stop
      chassis.Stop();
   }

   public void ParkInDepotLinear(boolean startsAtDucks, double speed){
      //should start along side wall north of warehouse with intake facing south
      //if startsAtDucks, skip next two steps
      if(!startsAtDucks) {
         //goToWall() if not already at it
         driveFuncs.GoToWall(90*sideMultiplier, speed);
         //wallFollow() to ducks on north wall
         driveFuncs.WallFollowForDuckDistance(speed,duckStopDistance);
      }
      //Go diagonal back towards middle of field for a time
      //Go north until against the north wall

      duckSpinner.Stop();

      //move to park
      driveFuncs.DriveForTime(90+(sideMultiplier*depotParkAngle),sideMultiplier*0.5,0,depotParkTime);

      driveFuncs.DriveForDistance(duckDistance,0,0.5,0,depotStopDistance);
      chassis.RawDrive(0,0,0);
   }

   public DuckPos ScanBarcodeOpenCV() throws InterruptedException {
      //get camera input and convert to mat
      //divide image into three sections
      //find section with most yellow
      DuckPos pos= DuckPos.NULL;
      opMode.telemetry.addData("Started scan", opMode.getRuntime());
      Bitmap in = camera.GetImage();
      in = camera.ShrinkBitmap(in,in.getWidth()/3,in.getHeight()/3);
      Mat img = camera.convertBitmapToMat(in);

      Rect firstRect = new Rect(0,0,img.width()/3,img.height());
      Rect secondRect = new Rect(img.width()/3,0,img.width()/3,img.height());
      Rect thirdRect = new Rect(2*img.width()/3,0,img.width()/3,img.height());

      Mat firstMat = new Mat(img,firstRect);
      Mat secondMat = new Mat(img,secondRect);
      Mat thirdMat = new Mat(img,thirdRect);
      //bgr lime(0,255,102)
      Scalar max = new Scalar(75,255,255);
      Scalar min = new Scalar(35,58,121);
      firstMat = camera.isolateColor(firstMat,max,min);
      secondMat = camera.isolateColor(secondMat,max,min);
      thirdMat = camera.isolateColor(thirdMat,max,min);
      Bitmap BigBit = camera.convertMatToBitMap(
              camera.isolateColor(
                      img
                      ,max,min)
      );


      Bitmap first = camera.convertMatToBitMap(firstMat);
      Bitmap second = camera.convertMatToBitMap(secondMat);
      Bitmap third = camera.convertMatToBitMap(thirdMat);

      //FtcDashboard.getInstance().sendImage(first);
      //FtcDashboard.getInstance().sendImage(second);
      //FtcDashboard.getInstance().sendImage(third);
      FtcDashboard.getInstance().sendImage(BigBit);

      double pixelsFirst = camera.countPixels(first);
      double pixelsSecond = camera.countPixels(second);
      double pixelsThird = camera.countPixels(third);

      opMode.telemetry.addData("Pixel Count 1",pixelsFirst);
      opMode.telemetry.addData("Pixel Count 2",pixelsSecond);
      opMode.telemetry.addData("Pixel Count 3",pixelsThird);


      if(pixelsFirst>pixelsSecond&&pixelsFirst>pixelsThird){
         pos=DuckPos.FIRST;
         opMode.telemetry.addData("Element in position","1");
      }
      else if(pixelsSecond>pixelsFirst&&pixelsSecond>pixelsThird){
         pos=DuckPos.SECOND;
         opMode.telemetry.addData("Element in position","2");
      }
      else if(pixelsThird>pixelsSecond&&pixelsThird>pixelsFirst){
         pos=DuckPos.THIRD;
         opMode.telemetry.addData("Element in position","3");
      }
      else if(pos==DuckPos.NULL) {
         opMode.telemetry.addData("Element in position", "null");
      }
      opMode.telemetry.addData("Scan Done!", opMode.getRuntime());
      return pos;
   }

   public void GoToPlaceLinear(){
      //Turn to zero
      driveFuncs.TurnToAngle(0,0.8);
      driveFuncs.TurnToAngle(0,0.2);
      //Go to the wall
      driveFuncs.GoToWall(1);
      //Wall follow the short distance to the white line
      driveFuncs.WallFollowToWhite(0.6,0);
      //Wall follow past the line
      driveFuncs.WallFollowForTime(1,0.25);
      //Dead reckon towards hub while turning
      driveFuncs.DriveForTimeToAngle(45*sideMultiplier,1,goToPlaceFacingAngle*sideMultiplier,turnWhileDrivingCoefficient,goToPlaceTime);
      //Turn to face hub
      driveFuncs.TurnToAngle(goToPlaceFacingAngle*sideMultiplier,0.5);
      //Go forwards a bit
      //DriveForTime(90*sideMultiplier,0.5,0,0.25);
   }

   public void GoToFreightLinear(){
      //should start north of white line along wall or at it
      //goToWall()
      //wallFollowToWhite() from the north
      //wallFollow() into the depot
      //select a freight to zeroIn() on
      //autoIntake() while zeroIn()-ing on freight
      //once freight is collected, goToWall() at a diagonal
      //wallFollowToWhite() from the south

        /*//Turn to zero
        TurnToAngle(0,0.4);
        //Go towards the wall at an angle
        GoToWall(-120*sideMultiplier,1);*/
      driveFuncs.GoToWallTurning(-120*sideMultiplier,1,0,turnWhileDrivingCoefficient);
      //Reset arm
      arm.ReturnToHomeAndIntake();
      //Wall follow to white line
      driveFuncs.WallFollowToWhite(0.6,180);
      //Go a little further
      driveFuncs. WallFollowForTime(-0.6,0.25);
   }

   public void AutoCollectFreightLinear(){
      //starts in warehouse along wall facing freight
      //turns on intake
      arm.ReturnToHomeAndIntake();
      //move forwards slowly while auto-intaking
      while (arm.intakeState == 1 && navigatorRunning){
         chassis.RawDrive(180,0.2,0);
      }
   }

   /*public void TurnToHubLinear() throws InterruptedException {
      //start facing hub
      //find sector of image with hub
      //move towards it

      boolean hDone = false;
      boolean right = false;
      boolean left = false;
      while(!hDone&&navigatorRunning){
         Bitmap img = camera.GetImage();
         if(side==AllianceSide.BLUE) {
            img = camera.convertMatToBitMap(camera.IsolateBlue(camera.convertBitmapToMat(img)));
         }else{
            img = camera.convertMatToBitMap(camera.IsolateRed(camera.convertBitmapToMat(img)));
         }
         FtcDashboard.getInstance().sendImage(img);
         img = camera.ShrinkBitmap(img,20,20);
         //FtcDashboard.getInstance().sendImage(camera.GrowBitmap(img,200,200));
         int[] vals = camera.findColor(img);
         if(vals[0]==-1)
         {
            if(side==AllianceSide.BLUE) {
               chassis.RawTurn(0.2);
               right=true;
            } else{
               chassis.RawTurn(-0.2);
               left=true;
            }
         }
         if(!hDone&&vals[0]!=-1) {
            if (vals[0] < 10) {
               opMode.telemetry.addData("turning","left");
               chassis.RawTurn(0.2);
               if (right) {
                  hDone = true;
                  chassis.RawTurn(0);
               }
               left=true;
            } else if (vals[0] >= 10) {
               opMode.telemetry.addData("turning","right");
               chassis.RawTurn(-0.2);
               if (left) {
                  hDone = true;
                  chassis.RawTurn(0);
               }
               right = true;
            }
         }
         opMode.telemetry.update();
      }
   }*/

   /*public void PlaceLinear() throws InterruptedException {
      while (levelSensor.getDistance(DistanceUnit.CM) > placeHeightThreshold&& navigatorRunning){ //while not above hub
         Bitmap img = camera.GetImage();
         if(side==AllianceSide.BLUE) {
            img = camera.convertMatToBitMap(camera.IsolateBlue(camera.convertBitmapToMat(img)));
         }else{
            img = camera.convertMatToBitMap(camera.IsolateRed(camera.convertBitmapToMat(img)));
         }
         //FtcDashboard.getInstance().sendImage(img);
         img = camera.ShrinkBitmap(img,20,20);
         FtcDashboard.getInstance().sendImage(camera.GrowBitmap(img,200,200));
         int[] vals = camera.findColor(img);
         opMode.telemetry.addData("distance",levelSensor.getDistance((DistanceUnit.CM)));
         opMode.telemetry.update();
         double error = vals[0]-10; //need to get error
         double offset = error * placeTurningCoefficient;
         chassis.RawDrive(chassis.GetImu().GetRobotAngle(), -placeSpeed, -offset); //drive forwards towards hub
      }
      chassis.Stop(); //stop
      arm.SetIntakeSpeed(-1); //reverse intake
      while (intakeDistance.getDistance(DistanceUnit.CM) < arm.GetArmIntakeDist() && navigatorRunning){ //while freight is still in intake
         //wait
      }
      arm.SetIntakeSpeed(0);//stop intake

   }*/

   ////MINOR FUNCTIONS////

   //Spine the ducks
   public void SpinDucks(double multiplier, double maxSpeed){
      NavigatorOn();
      lights.Purple();
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
      tele.addData("Duck Distance", duckDistance.getDistance(DistanceUnit.CM));
      tele.addData("Intake Distance", intakeDistance.getDistance(DistanceUnit.CM));
      tele.addData("Port Distance", portDist.getDistance(DistanceUnit.CM));
      tele.addData("Starboard Distance", starboardDist.getDistance(DistanceUnit.CM));
      tele.addData("Color Alpha", colorSensor.alpha());
   }


}
