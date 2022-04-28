package org.firstinspires.ftc.teamcode.Navigation.OpenCV;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "OpenCV Test", group = "Testing")
@Config
@Disabled

public class OpenCVTest extends LinearOpMode {
    private Pipeline pline;
    private OpenCVFrameGenerator fgen;
    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(new Pipeline());
        Mat results = new Mat();

        waitForStart();
        while(opModeIsActive()){
            int newresults = pline.GetResults();
            telemetry.addData("value: ",newresults);
            //Bitmap bmp = fgen.convertMatToBitMap(results);

        }





    }
}
