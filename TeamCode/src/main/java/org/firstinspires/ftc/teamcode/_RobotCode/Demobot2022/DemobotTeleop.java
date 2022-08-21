package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;


@TeleOp(name = "*DEMOBOT TELEOP*", group = "Demobot")
@Config
public class DemobotTeleop extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private Demobot robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns
    public static double odometryTestSpeed = -0.5;
    public static double odometryTestAngle = 180;
    public static double odometryTestX = 12;
    public static double odometryTestY = 12;

    private double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;



    @Override
    public void init() {
        robot = new Demobot(this,true,true,true);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        //hardwareMap.dcMotor.get("FR").setDirection(DcMotorSimple.Direction.REVERSE);
        //hardwareMap.dcMotor.get("FL").setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        msStuckDetectLoop = 5000;
    }

    @Override
    public void start(){
        robot.start();
        robot.getChassis().resetGyro();
        //if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE) robot.SetInputOffset(90); //90 is blue, -90 is red
        //else if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.RED) robot.SetInputOffset(-90); //90 is blue, -90 is red
        robot.getChassis().setHeadlessMode(true);
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        controllerInput2.Loop();

        ////PRINT TELEMETRY////
        //CONTROLS
        telemetry.addLine("----CONTROLS----");
        telemetry.addData("Drive with: ", "LJS");
        telemetry.addData("Turn with: ", "RJS");
        telemetry.addData("Change speed multiplier: ", "A");
        telemetry.addData("Reset robot angle: ", "Press RJS");
        telemetry.addData("Toggle headless mode: ", "Press LJS");
        telemetry.addData("Reset robot pose:", "Press B");
        telemetry.addData("Turn towards "+odometryTestAngle+" degrees:", "Hold Right Trigger");
        telemetry.addData("Move towards ("+odometryTestX+", "+odometryTestY+"):", "Hold Right Bumper");
        telemetry.addData("Move towards ("+odometryTestX+", "+odometryTestY+", "+odometryTestAngle+"):", "Hold Left Trigger");
        //DATA
        telemetry.addLine();
        telemetry.addLine("----DATA----");
        //Dead wheel positions
        telemetry.addLine("Dead wheel positions");
        double[] deadWheelPositions = robot.getNavigator().getDeadWheelPositions();
        telemetry.addData("LEFT dead wheel: ", deadWheelPositions[0]+" inches");
        telemetry.addData("RIGHT dead wheel: ", deadWheelPositions[1]+" inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2]+" inches");
        //Odometry estimated pose
        telemetry.addLine();
        telemetry.addLine("Robot pose");
        Pose2d robotPose = robot.getNavigator().getPose();
        telemetry.addData("X, Y, Angle", robotPose.getX() + ", " + robotPose.getY() + ", " + Math.toDegrees(robotPose.getHeading()));
        telemetry.addLine();

        //KILL SWITCH FOR NAVIGATOR
        if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {

        }

        robot.update();

        //Manage driving
        robot.getChassis().driveWithGamepad(controllerInput1, driveSpeed, turnSpeed, speedMultiplier);

        telemetry.update();
    }

    @Override
    public void stop(){
        robot.stop();
    }

    ////INPUT MAPPING////
    @Override
    public void ButtonPressed(int id, ControllerInput.Button button) {
        //Colored buttons
        if(button == Button.A){
            if (speedMultiplier == 1) speedMultiplier = 0.5;
            else speedMultiplier = 1; }
        if(button == Button.B) { //reset robot pose
            robot.getNavigator().setRobotPose(0, 0, 0);
            robot.getNavigator().getChassis().driveMotors.StopAndResetEncoders();
        }
        if(button == Button.X && robot.USE_PAYLOAD){
            /*Runnable runnable = () -> { robot.getPayload().Intake(); };
            Thread myTestThread = new Thread(runnable);
            myTestThread.start();*/
        }
        if(button == Button.Y && robot.USE_PAYLOAD);
        //Bumpers
        if(button == Button.LB);
        if(button == Button.RB);
        //Triggers
        if(button == Button.LT) robot.getNavigator().resetTurnPID();
        if(button == Button.RT) robot.getNavigator().resetTurnPID();
        //Dpad
        if(button == Button.DUP && robot.USE_PAYLOAD);
        if(button == Button.DDOWN && robot.USE_PAYLOAD);
        if(button == Button.DLEFT);
        if(button == Button.DRIGHT);
        //Joystick Buttons
        if(button == Button.LJS) robot.getChassis().switchHeadlessMode();
        if(button == Button.RJS && robot.USE_CHASSIS) robot.getChassis().resetGyro();
    }

    @Override
    public void ButtonHeld(int id, ControllerInput.Button button) {
        telemetry.addLine("A button is being held!");
        if(button == Button.RB) telemetry.addLine("RB held!");
        //Colored buttons
        if(button == Button.A);
        if(button == Button.B);
        if(button == Button.X);
        if(button == Button.Y);
        //Bumpers
        if(button == Button.LB);
        if (button == Button.RB) {
            boolean reached = robot.getNavigator().moveTowards(odometryTestX, odometryTestY, odometryTestSpeed);
            telemetry.addData("Drive towards ("+odometryTestX+", "+odometryTestY+") reached: ", reached);
        }
        //Triggers
        if(button == Button.LT){
            boolean reached = robot.getNavigator().goTowardsPose(odometryTestX, odometryTestY, odometryTestAngle, odometryTestSpeed);
            telemetry.addData("Drive towards ("+odometryTestX+", "+odometryTestY+", "+odometryTestAngle+") reached: ", reached);
        }
        if (button == Button.RT) {
            boolean reached = robot.getNavigator().turnTowards(odometryTestAngle, odometryTestSpeed);
            telemetry.addData("Turn towards "+odometryTestAngle+ " reached: ", reached);
        }
        //Dpad
        if(button == Button.DUP);
        if(button == Button.DDOWN);
        if(button == Button.DLEFT);
        if(button == Button.DRIGHT);
        //Joystick Buttons
        if(button == Button.LJS);
        if(button == Button.RJS);
    }

    @Override
    public void ButtonReleased(int id, ControllerInput.Button button) {
        //Colored buttons
        if(button == Button.A);
        if(button == Button.B);
        if(button == Button.X);
        if(button == Button.Y);
        //Bumpers
        if(button == Button.LB && robot.USE_PAYLOAD) ;
        if(button == Button.RB && robot.USE_PAYLOAD);
        //Triggers
        if(button == Button.LT && robot.USE_PAYLOAD);
        if(button == Button.RT && robot.USE_PAYLOAD);
        //Dpad
        if(button == Button.DUP);
        if(button == Button.DDOWN);
        if(button == Button.DLEFT);
        if(button == Button.DRIGHT);
        //Joystick Buttons
        if(button == Button.LJS);
        if(button == Button.RJS);
    }


}
