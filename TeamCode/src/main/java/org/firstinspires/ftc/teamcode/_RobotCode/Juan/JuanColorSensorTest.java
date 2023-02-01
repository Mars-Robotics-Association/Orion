package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "*JUAN COLOR SENSOR TEST*", group = "JUAN")
@Config
public class JuanColorSensorTest extends OpMode
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

    private ColorSensor colorSensor;

    @Override
    public void init() {
        robot = new Juan(this,false,true,true);
        robot.init();
        //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        telemetry.update();
    }

    @Override
    public void start(){
        robot.start();
        robot.getChassis().resetGyro();
        robot.getChassis().setHeadlessMode(false);
    }

    @Override
    public void loop() {
        telemetry.addData("Version", Juan.VERSION);

        int     r = colorSensor.red(),
                g = colorSensor.green(),
                b = colorSensor.blue();

        JuanAutonomousColorSensor.Color color = JuanAutonomousColorSensor.Color.scan(r, g, b);

        telemetry.addData("Color Detected", color.name());

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