package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@TeleOp(name = "*JUAN SCANNER TEST*", group = "JUAN")
@Config
public class JuanScannerTest extends OpMode
{
    ////Dependencies////
    private Juan robot;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns
    private final double speedMultiplier = 1;

    private final double liftOverrideSpeed = 5;

    public static int payloadControllerNumber = 1;

    private JuanPayload.SleeveScanner scanner;

    @Override
    public void init() {
        robot = new Juan(this,true,true,true);

        scanner = robot.getPayload().getScanner();

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
        telemetry.addData("Version", Juan.VERSION);

        try {
            scanner.runScan();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //update robot
        robot.update();

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

        if(robot.USE_PAYLOAD)robot.getPayload().printTelemetry();
    }

    @Override
    public void stop(){
        robot.stop();
    }
}
