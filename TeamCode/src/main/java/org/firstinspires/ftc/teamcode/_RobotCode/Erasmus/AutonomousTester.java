/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 */
@Config
@Autonomous(name="Erasmus Auto TESTER", group="Erasmus")
//@Disabled
public class AutonomousTester extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    ////Dependencies////
    private ErasmusRobot robot;
    private ControllerInput controllerInput1;
    private ControllerInput controllerInput2;
    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns

    private double speedMultiplier = 1;

    public static int payloadControllerNumber = 1;
    //i am going to be using a dc motor and its name is going to be armMotor
    public DcMotor armMotor;

    public static double armPower = 0.5;

    // Variables specific to autonomous
    public int signalResult = 0;
    public static double FORWARDSPEED = 0.5 ;
    public static double STRAFESPEED = 0.6 ;
    public static double TURNSPEED = 0.4 ;
    public static boolean DELIVERING = false ;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed Multiplier", speedMultiplier);

        robot = new ErasmusRobot(this, true, true, true);
        armMotor = hardwareMap.dcMotor.get("armMotor");
        robot.armTarget = robot.armBottom;
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.start();
        robot.getChassis().resetGyro();
        robot.getChassis().setHeadlessMode(true);

        // Initialization actions
        robot.openGripper();


        // ================= Wait for the game to start (driver presses PLAY) =======================
        waitForStart();
        runtime.reset();
        // ================== This is where our autonomous sequences happen =====================
        // V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V
        robot.closeGripper(0.2);
        //robot.armTarget = robot.armMid ;
        // ----------------------- Read signal ---------------------------
        goTo(23, 0, 0, FORWARDSPEED) ;          // Drive forward toward signal



        // Keep telemetry going till the end
        robot.stop();
        robot.armTarget = robot.armBottom;
        while (opModeIsActive()) {
            robot.update();
            printTelemetry();
        }
        robot.stop();
    }
    // ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^
    // ===========================================================================================


    // ========================== Support Methods =================================
    private void printTelemetry() {
        //DATA
        //Odometry estimated pose
        telemetry.addLine("----Robot pose----");
        Pose2d robotPose = robot.getNavigator().getMeasuredPose();
        telemetry.addData("X, Y, Angle", Math.round(robotPose.getX() * 100) / 100 + ", "
                + Math.round(robotPose.getY() * 100) / 100 + ", "
                + Math.round(Math.toDegrees(robotPose.getHeading()) * 100) / 100);
        telemetry.addLine("----DATA----");
        //telemetry.addData("red   ", String.valueOf(robot.colorSensor.red())) ;
        //telemetry.addData("green ", String.valueOf(robot.colorSensor.green())) ;
        //telemetry.addData("blue  ", String.valueOf(robot.colorSensor.blue())) ;
        //telemetry.addData("RSV   ", robot.hsvValues[0]);
        telemetry.addData("Signal   ", signalResult);
        telemetry.addData("Gripper: ", robot.servoTarget);
        telemetry.addData("Arm:     ", armMotor.getCurrentPosition());
        //Dead wheel positions
        /*telemetry.addLine("-----Dead wheel positions----");
        double[] deadWheelPositions = robot.getNavigator().getDeadWheelPositions();
        telemetry.addData("LEFT dead wheel:       ", deadWheelPositions[0]+" inches");
        telemetry.addData("RIGHT dead wheel:      ", deadWheelPositions[1]+" inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2]+" inches");
        */
        telemetry.update();
    }

    public void goTo( double destinationX, double destinationY, double destinationHeading, double speed ) {
        robot.update();
        while (!robot.getNavigator().goTowardsPose(destinationX, destinationY, destinationHeading, speed) && opModeIsActive()) {
            //while (!robot.getNavigator().goTowardsPose(destinationX, destinationY, destinationHeading, speed) && opModeIsActive()) {
            robot.update();
            printTelemetry();
        }
        robot.stop() ;
    }

    public void turnTo( double destinationHeading, double speed) {
        while (!robot.getNavigator().turnTowards(destinationHeading, speed) && opModeIsActive()) {
            robot.update();
            printTelemetry();
        }
        robot.stop() ;
    }

    //Wait for a period of time (seconds)
    public void waitForTime(double waitTimeSec) {
        double startTime = runtime.seconds();
        while (runtime.seconds()<startTime+waitTimeSec){
            robot.update() ;
        }
    }

}
