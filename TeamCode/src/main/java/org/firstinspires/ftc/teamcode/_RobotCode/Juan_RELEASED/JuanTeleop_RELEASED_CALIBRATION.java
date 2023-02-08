package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;

@TeleOp(name = "**JUAN TELEOP (TESTING 1.16.8)**", group = "OPPY")
@Config
public class JuanTeleop_RELEASED_CALIBRATION extends OpMode implements ControllerInputListener
{
    ////Dependencies////
    private Juan_RELEASED robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;

    private boolean IsBusy = false;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns
    private final double speedMultiplier = 1;

    private final double liftOverrideSpeed = 5;

    public static int payloadControllerNumber = 1;

    private JuanPayload_RELEASED payload;

    @Override
    public void init() {
        robot = new Juan_RELEASED(this,true,true,true);
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        controllerInput2 = new ControllerInput(gamepad2, 2);
        controllerInput2.addListener(this);

        msStuckDetectLoop = 30000;

        payload = robot.getPayload();

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();
    }

    @Override
    public void start(){
        robot.start();
        robot.getChassis().resetGyro();
        robot.getChassis().setHeadlessMode(false);
    }

    boolean lastRun = false;

    @Override
    public void loop() {
        telemetry.addData("Version", Juan_RELEASED.VERSION);

        MecanumChassis chassis = robot.getChassis();

        controllerInput1.loop();
        controllerInput2.loop();

        //update robot
        robot.update();
        //manage driving

        boolean isP2 = controllerInput2.calculateLJSMag() > 0.1 || Math.abs(controllerInput2.getRJSX()) > 0.1;

        if( !IsBusy) {
            if (isP2) {
                chassis.driveWithGamepad(controllerInput2, 1);
            } else {
                chassis.driveWithGamepad(controllerInput1, 1);
            }
        }

        telemetry.addData("Gripper Position", robot.getPayload().getGripper().getPosition());

        int direction = 0;
        if(gamepad1.dpad_up || gamepad2.dpad_up)direction++;
        if(gamepad1.dpad_down || gamepad2.dpad_down)direction--;
        if(lastRun || direction != 0)
            robot.getPayload().getLift().manualMove(direction);

        lastRun = direction != 0;

        //telemetry
        printTelemetry();
        telemetry.update();
    }

    //prints a large amount of telemetry for the robot
    private void printTelemetry() {
        //CONTROLS
        telemetry.addLine("----CONTROLS----");
        telemetry.addData("Drive with: ", "LJS");
        telemetry.addData("Turn with: ", "RJS");
        telemetry.addData("Toggle headless mode: ", "Press LOGITECH");

        //DATA
        telemetry.addLine();
        telemetry.addLine("----DATA----");
        //runtime
        //telemetry.addData("Loop time ms: ", (getRuntime()-lastRuntime)*1000);
        //Dead wheel positions
        telemetry.addLine("Dead wheel positions");
        double[] deadWheelPositions = robot.getNavigator().getDeadWheelPositions();
        telemetry.addData("LEFT dead wheel: ", deadWheelPositions[0]+" inches");
        telemetry.addData("RIGHT dead wheel: ", deadWheelPositions[1]+" inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2]+" inches");
        //Odometry estimated pose
        telemetry.addLine();
        telemetry.addLine("Robot pose");
        Pose2d robotPose = robot.getNavigator().getMeasuredPose();
        telemetry.addData("X, Y, Angle", robotPose.getX() + ", " + robotPose.getY() + ", " + Math.toDegrees(robotPose.getHeading()));

        if(robot.USE_PAYLOAD)robot.getPayload().printTelemetry();
    }

    @Override
    public void stop(){
        robot.stop();
    }

    ////INPUT MAPPING////
    @Override
    public void ButtonPressed(int id, Button button) {
        if(!robot.USE_PAYLOAD)return;
            JuanPayload_RELEASED payload = robot.getPayload();
            JuanPayload_RELEASED.LiftController lift = payload.getLift();
            JuanPayload_RELEASED.GripperController gripper = payload.getGripper();

            switch (button) {
                case LB:
                    gripper.release();
                    break;
                case RB:
                    gripper.grab();
                    break;
                case LT:
                    gripper.releaseMore();
                    break;
                case RT:
                    gripper.grabMore();
                    break;
                case GUIDE:
                    lift.reset();
                    break;
                case A:
                    lift.goToPreset(JuanPayload_RELEASED.LiftHeight.BOTTOM);
                    break;
                case B:
                    lift.goToPreset(JuanPayload_RELEASED.LiftHeight.LOW);
                    break;
                case Y:
                    lift.goToPreset(JuanPayload_RELEASED.LiftHeight.MEDIUM);
                    break;
                case X:
                    lift.goToPreset(JuanPayload_RELEASED.LiftHeight.HIGH);
                case RJS:
                    simAutonmous();
//                    robot.getNavigator().setMeasuredPose(0, 0, 0);
//                    robot.getNavigator().getChassis().driveMotors.stopAndResetEncoders();
//                    robot.getChassis().resetGyro();
                    break;
            }

    }

    @Override
    public void ButtonHeld(int id, Button button) {
        switch (button) {

            case GUIDE:
                robot.getNavigator().turnTowards(180, 0.5);
                break;
        }
    }

    @Override
    public void ButtonReleased(int id, Button button) {
        switch (button){

        }
    }

    void simAutonmous() {
        // SIMULATE AUTONOMOUS RUN
        JuanPayload_RELEASED payload = robot.getPayload();
        JuanPayload_RELEASED.LiftController lift = payload.getLift();
        JuanPayload_RELEASED.GripperController gripper = payload.getGripper();

        // CLOSE Gripper
        gripper.release();

        // Go to edge of SQUARE 4
        //goToPoseNoTimer(59, 0, 0, 1);
        while(robot.navigator.goTowardsPose( 59, 0, 0, 1) ) {
            robot.update();
            telemetry.update();
        }

        // Set Lift height to Cruise
        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.CRUISE);

        // Turn to Terminal
        //goToPoseNoTimer(59, 0, -85, .5);
        while(robot.navigator.goTowardsPose( 59, 0, -85, .5) ) {
            robot.update();
            telemetry.update();
        }

        // Raise Lift to HIGH
        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.HIGH);

        // Wait for lift
        while ( java.lang.Math.abs( lift.getLiftPosition() - lift.getLiftCurrentTargetPosition() ) > 50 ) {
            robot.update();
            telemetry.update();
        }

        // Move over terminal
        //goToPoseNoTimer( 59, -3.5,-85,.5);
        while(robot.navigator.goTowardsPose( 59, -3.5, -85, .5) ) {
            robot.update();
            telemetry.update();
        }

        // Lower Lift to MEDIUM
        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.MEDIUM);

        // Release Grip
        gripper.grab();

        // Back away
        //goToPoseNoTimer (59, -1, -85, .5);
        while(robot.navigator.goTowardsPose( 59, -1.5, -85, .5) ) {
            robot.update();
            telemetry.update();
        }

        // Turn back to Stack
        //goToPoseNoTimer (59, 0, -180, .5);
        while(robot.navigator.goTowardsPose(59, -1.5, -180, .5) ) {
            robot.update();
            telemetry.update();
        }

//        sleep (1000);

        //goToPoseNoTimer (47, 0, -180, 1);
        while(robot.navigator.goTowardsPose(47, 0, -180, 1) ) {
            robot.update();
            telemetry.update();
        }

        // Drive to Stack 1
//        goToPoseNoTimer( 45,0,90,1 );

        // Go to center of SQUARE 2
        //goToPoseNoTimer( 26,0,0,1);

    }

    void goToPose(double x, double y, double angle, double speed) {
        while(robot.navigator.goTowardsPose(x,y,angle,speed) ) { //&& !isStopRequested()) {
            robot.update();
            telemetry.update();
        }

    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}
