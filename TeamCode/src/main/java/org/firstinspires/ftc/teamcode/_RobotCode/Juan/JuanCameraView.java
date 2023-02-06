package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Navigation.Camera;

@TeleOp(name = "*JUAN CAMERA VIEW*", group = "JUAN")
@Config
public class JuanCameraView extends OpMode
{
    ////Dependencies////
    private Camera camera;
    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns
    private final double speedMultiplier = 1;

    private JuanPayload.SleeveScanner scanner;

    @Override
    public void init() {
        camera = new Camera(this, "Webcam 1");

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();
    }

    @Override
    public void start(){
    }

    @Override
    public void loop() {
        telemetry.addData("Version", Juan.VERSION);

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
    }

    @Override
    public void stop(){}
}
