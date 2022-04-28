package org.firstinspires.ftc.teamcode.Navigation.OpenCV;

import android.graphics.Bitmap;

import java.io.ByteArrayOutputStream;

//Converts bitmap to base64 to send to GUI
public class EncodedImageRecognition {
    public String encodedImage;

    public EncodedImageRecognition(Bitmap bitmap) {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        bitmap.compress(Bitmap.CompressFormat.PNG, 100, baos);
        byte[] b = baos.toByteArray();
        encodedImage = android.util.Base64.encodeToString(b, android.util.Base64.DEFAULT);
    }
}
