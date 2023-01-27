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

@Autonomous(name = "Ingenuity Autonomous", group = "Ingenuity")
@Config
public class IngenuityAutonomous extends LinearOpMode
{
    public static double speed = 0.6;
    IngenuityPowerPlayBot robot;
    //public DcMotor armMotor ;
    IngenuityPowerPlayBot.SignalColor signalZone = IngenuityPowerPlayBot.SignalColor.GREEN;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityPowerPlayBot(this,true,true,true);
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

        while (arm.getPosition()<0.1){
            robot.update();
            telemetry.update();
        }

        //go forward to the signal
        goToPose(22,0,0);
        //read the signal
        signalZone=robot.readSignal();
        //wait
        sleep(250);

        // go dump the signal cone
        //goToPose(25,-10,-75);

        // try to score on the medium junction
        goToPose(26.5, -1, -45);
        dunkCone(arm);

        // get lined up for cone stack
        goToPose(50, 0, 87);

        //go to cone stack
        arm.goToPosition(0.0535);
        while (arm.getPosition() > 0.058) {
            robot.update();
            telemetry.update();
        }
        goToPose(50, 14.5, 87);
        sleep(250);

        robot.ensureGripperClosed();
        sleep(500);
        robot.moveArmToStop(2);
        goToPose(50, 8.5, 87);

        // line up for high junction
        robot.moveArmToStop(3);
        goToPose(50,-19,70);

        // advance on high junction
        goToPose(54, -17, 45);
        dunkCone(arm);

        // retreat from high junction
        goToPose(50, -21, 45);


        switch (signalZone) {
            case BLUE:
                goToPose(50, -24, 0);
                goToPose(25, -24, -120);
                break;
            case RED:
                goToPose(50, 0, 0);
                goToPose(25, 0, -90);
                break;
            default:
                goToPose(50, 24, 0);
                goToPose(25, -24, -90);
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

        while (!isStopRequested()){
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.update();
        }

        robot.stop();
    }

    private void dunkCone(EncoderActuator arm) {
        double armStartPos = arm.getPosition();
        arm.goToPosition(armStartPos - .025);
        sleep(250);
        robot.ensureGripperOpen();
        sleep(375);
        arm.goToPosition(armStartPos);
        sleep(500);
    }

    private void goToPose(double x, double y, double angle) {
        while( robot.getNavigator().goTowardsPose(x, y, angle,speed) && !isStopRequested()) {
            robot.update();
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.addData("Going to to ", "("+x+", "+y+", "+angle+")");
            telemetry.update();
        }
        robot.getChassis().stop();
    }
    private void turn(double angle) {
        while( robot.getNavigator().turnTowards(angle,speed) && !isStopRequested()) {
            robot.update();
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.update();
        }
    }

    public void autoDrive() {
        // Move in rectangle, clockwise around the post
        // Move forward to 16x, 0y
        goToPose(20,0,0);

        // Strafe right to 16v, 16y
        goToPose(20,20,0);

        // Move backward to 0x, 16y
        goToPose(0,20,0);

        // strafe left to 0x, 0y
        goToPose(0,0,0);
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
        telemetry.addData("LEFT dead wheel:       ", deadWheelPositions[0]+" inches");
        telemetry.addData("RIGHT dead wheel:      ", deadWheelPositions[1]+" inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2]+" inches");
        //Odometry estimated pose
        telemetry.addLine();
        telemetry.addLine("Robot pose");
        Pose2d robotPose = robot.getNavigator().getMeasuredPose();
        telemetry.addData("X, Y, Angle",
                Math.round(robotPose.getX()*100)/100
                        + ", " + Math.round(robotPose.getY()*100)/100 + ", "
                        + Math.round(Math.toDegrees(robotPose.getHeading())*100)/100);
        telemetry.addLine();
        telemetry.update();
    }


}
