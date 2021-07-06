package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Orion.OrionNavigator;

import static org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive.DriveConstants.MAX_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive.DriveConstants.MAX_ANG_ACCEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive.DriveConstants.MAX_ANG_VEL_MOD;
import static org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive.DriveConstants.MAX_VEL_MOD;

@Config
@Autonomous(name = "*CURIOSITY AUTO*", group = "Curiosity")
public class CuriosityAutonomous extends LinearOpMode {
    private CuriosityUltimateGoalControl control;
    private OrionNavigator orion;
    private FtcDashboard dashboard;

    public static double tfDistCoef = 6666;
    public static double tfXCoef = 0.001;

    public static double robotX = 0;
    public static double robotY = 4;
    public static double robotH = 180;

    public static double powerShotStartAngle = 2;
    public static double powerShotStartX = 60;
    public static double powerShotStartY = 36;
    public static double powerShotIncrament = -6;

    public static double ringDetectionAngle = 180 + 15;

    public static double tfUpperLimit = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        control = new CuriosityUltimateGoalControl(this,true,true,true);
        control.Init();
        orion = control.GetOrion();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        //set roadrunner speed modifiers
        if(control.isUSE_NAVIGATOR()){
            MAX_VEL_MOD  = 1;
            MAX_ACCEL_MOD  = 1;
            MAX_ANG_VEL_MOD  = 1;
            MAX_ANG_ACCEL_MOD = 1;
        }

        waitForStart();
        orion.SetPose(robotX, robotY, Math.toRadians(robotH));//robot starts on red right line

        //put the starpath in the right place
        control.StarpathToStorage();

        //turn to face discs
        orion.TurnTo(ringDetectionAngle);

        //wait a bit for it to see discs
        sleep(1500);//wait for tensorflow to detect discs
        int numberOfDiscs = orion.GetNumberOfDiscs(tfUpperLimit);//figure out where to go\

        //turn back to zero
        orion.TurnTo(180);

        //Go near square A
        orion.MoveSpline(40, 0, 0, true);

        if(numberOfDiscs == 0){ //A
            //deposit wobble goal
            orion.MoveLine(68, -12);
            orion.MoveLine(60, -12);
        }
        else if(numberOfDiscs > 0 && numberOfDiscs < 3){ //B
            //spline to B, deposit
            orion.MoveSpline(88, 12, 0, true);
        }
        else { //C
            //keep going forwards, deposit
            orion.MoveLine(115, -8);
        }

        //Line up for high goal
        control.ShooterOff();
        control.ShooterOn();
        orion.MoveLine(55, 20);
        orion.TurnTo(180);
        control.ShootThree();

        /*//Move to next wobble goal
        orion.MoveLine(55, -4, 0);
        orion.MoveLine(12, 4, 0);
        orion.TurnTo(270);
        orion.MoveLine(24, 30, 0);

        //Spine movement
        orion.MoveSpline(30, 45, -30, true);
        orion.TurnTo(180);
        orion.MoveSpline(60, 20, 0, true);*/


        orion.MoveLine(55, -4);
        orion.MoveLine(1, 4);
        orion.MoveLine(1, 16);
        orion.MoveLine(0, 24);

        orion.MoveSpline(40, 32, 0, true);

        if(numberOfDiscs == 0){ //A
            //deposit wobble goal
            orion.MoveSpline(65, 0, -45, true);
            orion.MoveLine(60, 0);
            orion.MoveLine(70, 5);
        }
        else if(numberOfDiscs > 0 && numberOfDiscs < 3){ //B
            //spline to B, deposit
            orion.MoveSpline(80, 12, 0, true);
            orion.MoveLine(75, 12);
            orion.MoveLine(70, 0);
        }
        else { //C
            //keep going forwards, deposit
            orion.MoveSpline(102, -8, 0, true);
            orion.MoveLine(100, -8);
            orion.MoveLine(70, 0);
        }


    }

    private void HighGoalRoutine(){
        control.ShooterOn();
        control.RotateStarpathToNextPos();
        sleep(1500);
        control.RotateStarpathToNextPos();
        sleep(1500);
        control.RotateStarpathToNextPos();
        sleep(500);
        control.ShooterOff();
    }

    private void PowerShotRoutine(){
        //Start at (x,y)
        control.ModifyForPowerShot();
        control.ShooterOn();

        //turn to first shot
        orion.Turn(powerShotStartAngle);
        control.ShootOne();
        control.ShooterOn();

        control.ModifyForPowerShot();
        //turn to second shot
        orion.Turn(powerShotIncrament);
        //orion.MoveLine(powerShotStartX, powerShotStartY+powerShotIncrament, 0);
        control.ShootOne();
        control.ShooterOn();

        control.ModifyForPowerShot();
        //turn to third shot
        orion.Turn(powerShotIncrament);
        //orion.MoveLine(powerShotStartX, powerShotStartY+powerShotIncrament+powerShotIncrament, 0);
        control.ShootOne();

        //Reset shooter
        control.ShooterOff();
        control.StopModifyForPowerShot();
    }
    /*private void Shoot(){
        control.ShootAsync();
        while(control.IsShooterRunning()){
            control.ShootAsync();
            telemetry.addLine("Shooting");
            telemetry.update();
        }
        control.StopShootAsync();
        control.ShooterOn();
    }*/
}

/*if(numberOfDiscs == 0){ //go to A
        telemetry.addLine("close target");
        orion.MoveSpline(30, 12, 0);//drop off wobble goal 1
        orion.MoveLine(-62, -36, 0);//go to second wobble goal
        orion.MoveLine(0, -24, 0);
        orion.MoveSpline(62, 60, 0);//places it
        }

        else if(numberOfDiscs > 0 && numberOfDiscs < 3){ //go to B
        telemetry.addLine("middle target");
        orion.MoveSpline(54, -2, 0);//drop off wobble goal 1
            *//*orion.MoveLine(-10, -30, 0);
            orion.MoveLine(-72, 0, 0);*//*
        orion.MoveLine(-44,-50, 0);//heads back
        orion.MoveLine(-40,0, 0);
        orion.MoveLine(0,20,0);//lines up for second wobble goal
        orion.MoveLine(30,-3,0);//places it
        orion.MoveSpline(56,10,0);
        orion.MoveLine(-15,0,0);//goes back to line

        }

        else{ //go to C
        telemetry.addLine("far target");
        orion.MoveSpline(78, 12, 0);//drop off wobble goal 1
        orion.MoveLine(-68,-52, 0);//heads back
        orion.MoveLine(-42,0, 0);
        orion.MoveLine(0,16,0);//lines up for second wobble goal
        orion.MoveLine(30,0,0);//places it
        orion.MoveSpline(80,34,0);
        }*/
