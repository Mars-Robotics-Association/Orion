package org.firstinspires.ftc.teamcode.Navigation.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;


//Contains all the thread code for the robot. Does NOT contain any actual logic or code which might need to be changed regularly

class FreightFrenzyNavigatorThread implements Runnable
{
    //dependencies
    OpMode opMode;
    BlinkinController lights;
    FreightFrenzyNavigator navigator;

    //thread
    Thread thread;
    boolean threadRunning = false;

    //function running in thread variables
    boolean startSpinDucks = false;
    boolean startParkWarehouse = false;
    boolean startParkDepot = false;
    boolean startScanBarcode = false;
    boolean startGoToPlace = false;
    boolean startGoToCollect = false;
    boolean startCollect = false;


    int currentNumberOfSpinCycles = 1;
    double currentParkFurtherInTime = 2;
    boolean currentStartAtDucks = false;
    boolean currentParkFurtherInWarehouse = false;
    double currentRobotSpeed = 0.5;

    public FreightFrenzyNavigatorThread(OpMode setOpMode, BlinkinController setLights, FreightFrenzyNavigator setNavigator){
        opMode = setOpMode;
        lights = setLights;
        navigator = setNavigator;
        thread = new Thread(this);
    }

    ////THREAD CODE////

    public void SetThread(Thread setThread) {thread=setThread;}

    //Runs in seperate thread
    @Override
    public void run() {
        opMode.telemetry.addLine("Nav Thread Start!");
        threadRunning = true;
        navigator.NavigatorOn();
        lights.Purple();

        if(startSpinDucks) {
            startSpinDucks = false;
            navigator.DriveAndSpinDucksLinear(currentNumberOfSpinCycles,currentRobotSpeed);
        }
        if(startParkWarehouse) {
            startParkWarehouse = false;
            navigator.ParkInWarehouseLinear(currentRobotSpeed,currentParkFurtherInWarehouse);
        }
        if(startParkDepot) {
            startParkDepot = false;
            navigator.ParkInDepotLinear(currentStartAtDucks,currentRobotSpeed);
        }
        if(startScanBarcode) {
            startScanBarcode = false;
            try {
                navigator.ScanBarcodeOpenCV();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if(startGoToPlace) {
            startGoToPlace = false;
            navigator.GoToPlaceLinear();
        }
        if(startGoToCollect) {
            startGoToCollect = false;
            navigator.GoToFreightLinear();
        }
        if(startCollect){
            startCollect = false;
            navigator.AutoCollectFreightLinear();
        }

        opMode.telemetry.addLine("Nav Thread End!");

        lights.Yellow();
        threadRunning = false;
    }
    public boolean IsThreadRunning(){return threadRunning;}


    ////START FUNCTIONS////
    public void StartSpinDucks(int numberOfCycles){
        currentNumberOfSpinCycles = numberOfCycles;
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
    public void StartGoToPlace(){
        startGoToPlace = true;
        thread.start();
    }
    public void StarGoToCollect(){
        startGoToCollect = true;
        thread.start();
    }
    public void StartCollecting(double armRaiseHeight){
        navigator.SetArmAutoHeight(armRaiseHeight);
        startCollect = true;
        thread.start();
    }
}
