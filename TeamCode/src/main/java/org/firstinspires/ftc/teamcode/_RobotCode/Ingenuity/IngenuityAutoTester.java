package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;

@Autonomous(name = "Ingy Auto Tester", group = "Ingenuity")
@Config
public class IngenuityAutoTester extends LinearOpMode
{
    public static double speed = 0.5;
    IngenuityPowerPlayBot robot;
    //public DcMotor armMotor ;
    IngenuityPowerPlayBot.SignalColor signalZone = IngenuityPowerPlayBot.SignalColor.GREEN;
    private double[] armStops = {0.0, 0.1655, 0.23177, 0.3476} ;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityPowerPlayBot(this,true,true,true);
        robot.init();
        robot.getChassis().setHeadlessMode(true);

        EncoderActuator arm = robot.getPayload().getArm();

        // ===================================================================
        waitForStart();
        robot.start();
        // ===================================================================
        robot.closeGripper() ;
        sleep(500) ;
        arm.goToPosition(armStops[3]) ;
        while (arm.getPosition()<0.2){
            robot.update();
            telemetry.update();
        }

        goToPose(22,0,0);    //go forward to the signal
        sleep(200);    // pause to ensure reading
        signalZone=robot.readSignal();    //read the signal

        goToPose(40,0,0); //move almost to next tile (away from us)
        goToPose(57,5,45) ; // Drive to place the cone on high junction
        sleep(500);
        robot.openGripper() ; // release cone on high junction
        sleep(500);
        goToPose(49,0,0) ; // Back away from high junction
        arm.goToPosition(armStops[0]+0.05) ; // Start lowering the arm  + offset for stack
        sleep(1000);

        goToPose(50,-20,-90) ; // Drive to depot
        robot.closeGripper() ;
        sleep(500) ;
        arm.goToPosition(armStops[1]) ; // Raise arm for low junction
        goToPose(50,0,-135) ; // Drive to place the cone on high junction
        robot.openGripper() ; // release cone on high junction
        sleep(500);

        turn(-90) ;
        arm.goToPosition(armStops[0]) ; // Start lowering the arm  + offset for stack
        //strafe to signal zone
        switch (signalZone) {
            case BLUE:
                goToPose(50, -23, -90);
                break;
            case RED:
                goToPose(50, 0, -90);
                break;
            default:
                goToPose(50, 23, -90);
        }


        while (!isStopRequested()){
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.update();
        }

        robot.stop();
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

    private void printTelemetry() {

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
