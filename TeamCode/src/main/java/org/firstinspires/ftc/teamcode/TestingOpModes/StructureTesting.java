package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Orion.NavModules.*;
@TeleOp(name="StructureTesting")
public class StructureTesting extends LinearOpMode {
    Camera camera = new Camera(this,"Webcam 1");

    @Override
    public void runOpMode() throws InterruptedException {
        camera.PrintTFTelemetry();
    }
}
