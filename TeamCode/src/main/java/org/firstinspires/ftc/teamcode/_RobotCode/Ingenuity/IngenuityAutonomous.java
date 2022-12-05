package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.Path;
import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.PathPoint;

@Autonomous(name = "Ingenuity Autonomous", group = "Ingenuity")
@Config
public class IngenuityAutonomous extends LinearOpMode
{
    public static double speed = 0.3;
    IngenuityPowerPlayBot robot;
    //public DcMotor armMotor ;
    int signalZone = 0;


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
        while(! robot.getNavigator().goTowardsPose(18,3,0,0.22)&& !isStopRequested()) {
            robot.update();
            printTelemetry();

        //go forward to the signal
        goToPose(21,0,0);
        //read the signal
        signalZone=robot.readSignal();
        //move forward
        goToPose(25,0,0) ;
        //strafe to signal zone
        switch(signalZone){
            case 1 :
                goToPose(23,-25,0);
                break;
            case 3:
                goToPose(25,15,0);
                break;
            default:
                goToPose(23,0,0);
        }

        // Move in rectangle, clockwise around the post
        // Move forward to 16x, 0y

        // Strafe right to 16v, 16y
        //goToPose(20,20,0);

        // Move backward to 0x, 16y
        //goToPose(0,20,0);

        // strafe left to 0x, 0y
        //goToPose(0,0,0);

        //autoDrive();

        robot.stop();
    }

    private void goToPose(double x, double y, double angle) {
        while(! robot.getNavigator().goTowardsPose(x, y, angle,0.25) && !isStopRequested()) {
            robot.update();
            robot.opMode.telemetry.addData("Going to to ", "("+x+", "+y+", "+angle+")");
            robot.opMode.telemetry.update();
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
        telemetry.addData("color ",String.valueOf(robot.colorSensor.red()),String.valueOf(robot.colorSensor.green()),String.valueOf(robot.colorSensor.blue()));
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
        telemetry.addData("X, Y, Angle", robotPose.getX() + ", " + robotPose.getY() + ", " + Math.toDegrees(robotPose.getHeading()));
        telemetry.addLine();
        telemetry.update();
    }


}
