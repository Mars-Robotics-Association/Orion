package org.firstinspires.ftc.teamcode.OpenCV;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.Base64;

public class EncodedImageRecognition {
    public String encodedImage;

    public EncodedImageRecognition(Bitmap bitmap) {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        bitmap.compress(Bitmap.CompressFormat.PNG, 100, baos);
        byte[] b = baos.toByteArray();
        encodedImage = android.util.Base64.encodeToString(b, android.util.Base64.DEFAULT);
    }
}
