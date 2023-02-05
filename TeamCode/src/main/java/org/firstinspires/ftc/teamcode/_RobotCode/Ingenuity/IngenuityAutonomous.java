package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.Path;
import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.PathPoint;

import java.util.Date;

@Autonomous(name = "Ingenuity Autonomous", group = "Ingenuity")
@Config
public class IngenuityAutonomous extends LinearOpMode {
    public static double speed = 0.6;
    IngenuityPowerPlayBot robot;
    //public DcMotor armMotor ;
    IngenuityPowerPlayBot.SignalColor signalZone = IngenuityPowerPlayBot.SignalColor.GREEN;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityPowerPlayBot(this, true, true, true);
        robot.init();
        robot.getChassis().setHeadlessMode(true);
        //armMotor = hardwareMap.dcMotor.get("armMotor") ;
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
        //armMotor.setDirection(DcMotorSimple.Direction.REVERSE) ;

        waitForStart();
        robot.start();

        EncoderActuator arm = robot.getPayload().getArm();

        robot.ensureGripperClosed();
        sleep(500);
        robot.moveArmToStop(2);

        while (arm.getPosition() < 0.1) {
            robot.update();
            telemetry.update();
        }

        //scoot left a little
        goToPose(2, -4, 0, 1000, false);
        //go forward to the signal
        goToPose(22, -2, 0, 10000, true);
        //read the signal
        signalZone = robot.readSignal();
        //wait
        sleep(50);

        // try to score on the medium junction
        goToPose(26.5, -1.5, -45, 10000, true);
        dunkCone(arm);

        // get lined up for cone stack
        goToPose(50, 0, 87, 2500, false);

        //go to cone stack
        arm.goToPosition(0.058);
        sleep(100);
        goToPose(50, 13, 87, 10000, true);
        sleep(250);

        // grab cone from stack, raise arm, and back up a little
        robot.ensureGripperClosed();
        sleep(400);
        robot.moveArmToStop(2);
        sleep(50);
        goToPose(50, 0, 87, 1200, false);

        // turn around to go to high junction
        robot.moveArmToStop(3);
        goToPose(54, -10.5, -36, 10000, true);


//        // line up for high junction
//        robot.moveArmToStop(3);
//        goToPose(50,-20,70,2500,false);
//
//        // advance on high junction
//        goToPose(54, -18, 45,10000,true);

        // drop cone on high junction
        dunkCone(arm);

        // straight back from high junction
        goToPose(49, -5, -36, 1000, false);


        // retreat from high junction
        //goToPose(48, -22, 45,10000,true);

        if (true) {
            arm.goToPosition(0.053);
            sleep(250);
            goToPose(50, 13, 87, 10000, true);
            sleep(250);

            robot.ensureGripperClosed();
            sleep(400);
            robot.moveArmToStop(2);
            sleep(50);
            goToPose(50, 6, 87, 1000, false);
            robot.moveArmToStop(1);
            goToPose(48, -4.5, 127, 10000, true);
            dunkCone(arm);
            robot.moveArmToStop(3);
        }

        switch (signalZone) {
            case RED:
                goToPose(50, 0, -175, 10000, true);
                break;
            case GREEN:
                goToPose(50, 18, -178, 3000, true);
                break;
            default:
                goToPose(48, -23, -179, 10000, true);
        }
//        arm.goToPosition(0);
//        while (arm.getPosition()>0.05){
//            robot.update();
//            telemetry.update();
//        }
//
        sleep(200);

        robot.moveArmToStop(0);
        robot.ensureGripperOpen();

        while (!isStopRequested()) {
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.update();
        }

        robot.stop();
    }

    private void dunkCone(EncoderActuator arm) {
        double armStartPos = arm.getPosition();
        arm.goToPosition(armStartPos - .03);
        sleep(200);
        robot.ensureGripperOpen();
        sleep(325);
        arm.goToPosition(armStartPos);
        sleep(400);
    }

    private void goToPose(double x, double y, double angle, int bailTime, boolean stopAtEnd) {
        double started = this.time;
        int msElapsed = 0;
        while (robot.getNavigator().goTowardsPose(x, y, angle, speed) && !isStopRequested()) {
            if ((this.time - started) * 1000 > bailTime) break;
            robot.update();
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.addData("Going to to ", "(" + x + ", " + y + ", " + angle + ")");
            telemetry.update();

        }
        if (stopAtEnd) {
            robot.getChassis().stop();
        }
    }

    private void turn(double angle) {
        while (robot.getNavigator().turnTowards(angle, speed) && !isStopRequested()) {
            robot.update();
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.update();
        }
    }

    public void autoDrive() {
        // Move in rectangle, clockwise around the post
        // Move forward to 16x, 0y
        goToPose(20, 0, 0, 10000, true);

        // Strafe right to 16v, 16y
        goToPose(20, 20, 0, 10000, true);

        // Move backward to 0x, 16y
        goToPose(0, 20, 0, 10000, true);

        // strafe left to 0x, 0y
        goToPose(0, 0, 0, 10000, true);
    }

    private void printTelemetry() {
        /*
        //CONTROLS

        telemetry.addLine("----CONTROLS----");
        telemetry.addData("Drive with: ", "LJS");
        telemetry.addData("Turn with: ", "RJS");
        telemetry.addData("Change speed multiplier: ", "A");
        telemetry.addData("Reset robot pose: ", "Press RJS");
        telemetry.addData("Toggle headless mode: ", "Press LJS");
        telemetry.addData("Intake: ", "Hold LT");
        telemetry.addData("Load: ", "Hold RT");
        telemetry.addData("Toggle shooter: ", "Press Y");
        telemetry.addData("Toggle intake: ", "Press RB");
        telemetry.addData("Toggle path: ", "Press LB");

        robot.getPayload().printTelemetry();
        */
        //DATA
        telemetry.addLine();
        telemetry.addData("Zone: ", signalZone);
        telemetry.addLine("----DATA----");
        telemetry.addData("Gripper: ", robot.servoTarget);
        //telemetry.addData("Arm:     ", armMotor.getCurrentPosition());
        //Dead wheel positions
        telemetry.addLine("Dead wheel positions");
        double[] deadWheelPositions = robot.getNavigator().getDeadWheelPositions();
        telemetry.addData("LEFT dead wheel:       ", deadWheelPositions[0] + " inches");
        telemetry.addData("RIGHT dead wheel:      ", deadWheelPositions[1] + " inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2] + " inches");
        //Odometry estimated pose
        telemetry.addLine();
        telemetry.addLine("Robot pose");
        Pose2d robotPose = robot.getNavigator().getMeasuredPose();
        telemetry.addData("X, Y, Angle",
                Math.round(robotPose.getX() * 100) / 100
                        + ", " + Math.round(robotPose.getY() * 100) / 100 + ", "
                        + Math.round(Math.toDegrees(robotPose.getHeading()) * 100) / 100);
        telemetry.addLine();
        telemetry.update();
    }


}
