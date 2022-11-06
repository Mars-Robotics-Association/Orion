package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode._RobotCode.Demobot2022.Demobot;

@Autonomous(name = "JUAN Test Autonomous", group = "JUAN")
@Config
public class JuanAutonomous extends LinearOpMode
{
    Demobot robot;
    Camera camera;

    @Override
    public void runOpMode() {
        robot = new Demobot(this,true,true,true);
        robot.init();
        camera = new Camera(this,"Webcam 1",false);

        waitForStart();
        robot.start();

        robot.getChassis().setHeadlessMode(true);

        switch(readConeState()){

        }
    }

    private ConeState readConeState(){
        return ConeState.A;
    }

    enum ConeState{
        A,
        B,
        C
    }
}
