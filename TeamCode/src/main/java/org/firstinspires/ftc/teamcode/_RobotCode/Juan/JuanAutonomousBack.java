package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.opencv.core.Mat;

@Autonomous(name = "JUAN BACK", group = "JUAN")
@Config
public class JuanAutonomousBack extends LinearOpMode
{
    ScanRect rectScanner;
    Juan robot;
    Camera camera;

    Mat cameraMat = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan(this,true,true, false);
        robot.init();
//        camera = new Camera(this,"Webcam 1",false);
//
//        rectScanner = new ScanRect(){{
//            int x1 = 0;
//            int y1 = 0;
//            int x2 = 0;
//            int y2 = 0;
//        }};
//
//        rectScanner.processFrame(camera.convertBitmapToMat(camera.GetImage()));

        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);

        robot.getChassis().rawDrive(0, 5, 0);
        sleep(500);
        robot.getChassis().rawDrive(0, 0, 0);
    }

    private ConeState readConeState(){

        return ConeState.A;
    }

    enum ConeState{
        A,
        B,
        C
    }

    void doSomething(int count) throws InterruptedException {
        MecanumChassis chassis = robot.getChassis();

        for(int i = 0; i < count; i++){
            chassis.rawTurn(5);
            Thread.sleep(500);
            chassis.rawTurn(0);
            Thread.sleep(500);
        }
    }

    private void PathA() throws InterruptedException{
        doSomething(1);
    }

    private void PathB() throws InterruptedException{
        doSomething(2);
    }

    private void PathC() throws InterruptedException{
        doSomething(3);
    }
}
