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


        //go forward to the signal
        goToPose(22,0,0);
        //read the signal
        signalZone=robot.readSignal();
        //wait
        sleep(200);
        //move forward
        goToPose(40,0,0);
        goToPose(51,0,45) ;
        sleep(1000);
        goToPose(51,-20,-90) ;
        goToPose(51,0,0) ;

        goToPose(26,0,0);
        //turn(90);
        //strafe to signal zone
        switch(signalZone){
            case BLUE:
                goToPose(25,-23,0);
                break;
            case GREEN:
                goToPose(25,23,0);
                break;
            default:
                goToPose(25,0,0);
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
