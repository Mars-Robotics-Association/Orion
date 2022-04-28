package org.firstinspires.ftc.teamcode.Navigation.OpenCV;


import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(name = "Frame Analysis", group = "Testing")
@Config
@Disabled

public class FrameAnalysis extends LinearOpMode {
    private VuforiaLocalizer vuforia;



    @Override
    public void runOpMode() throws InterruptedException {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);

        waitForStart();
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        Image img = frame.getImage(0);
        Bitmap bitmap = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.ARGB_8888);

    }
}
