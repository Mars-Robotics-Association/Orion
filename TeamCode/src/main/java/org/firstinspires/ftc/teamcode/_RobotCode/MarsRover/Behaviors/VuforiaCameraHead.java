package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import android.graphics.Bitmap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class VuforiaCameraHead extends Behavior {
    // put this somewhere safe please
    private static final String VUFORIA_LISCENCE_KEY = "AeZ+Eyv/////AAABmfcFKgZ5NkXfgqEeyUnIJMIHuzBJhNTY+sbZO+ChF7mbo1evegC5fObZ830PRBTGTM6jbp+1XXCzx3XhY1kaZevEQXNpAKhXU9We0AMlp1mhnAUjGI2sprJZqJIfFGxkK598u8Bj3qQy4+PlCrk+Od/tAGs8dqAAsZPp4KpczFQttxMBC5JZNeIbIFP57InXOeJgyeH1sXK+R2i6nPfOFRvHJjdQaLWdAasv7i3b0RH5ctG7Ky7J9g9BPYI03kkChCJkbPg03XnoqCcC7rEpAk3n8a9CqtwTUu57Sy0jCDUd2O6X9kHjZ5ZmS0I3O0YSzX3Jp2ppTE2kDS2I9zBYEmuEqkMjItxd52oES0Ij0rZm";

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private void initVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_LISCENCE_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private Mat currentCVFrame;
    public Mat getCVFrame(){return currentCVFrame;}

    // Yaw = left/right
    // Pitch = up/down

    private Servo yawServo;
    private Servo pitchServo;

    public double getYaw(){
        return yawServo.getPosition()*270;
    }

    public double getPitch(){
        return pitchServo.getPosition()*270;
    }

    public void setYaw(double degrees){
        yawServo.setPosition(degrees/270);
    }

    public void setPitch(double degrees){
        pitchServo.setPosition(degrees/270);
    }

    @Override
    protected void init() {
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }

    private List<Recognition> recognitions = new ArrayList<>();

    public List<Recognition> getRecognitions(){
        return recognitions;
    }

    @Override
    protected void start() {

    }

    @Override
    protected void update() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                recognitions = updatedRecognitions;
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                }
            }
        }else{
            RobotLog.e("TensorFlow has stopped unexpectedly");
            throw new Error("TensorFlow has stopped unexpectedly");
        }

        vuforia.getFrameBitmap(Continuation.create(ThreadPool.getDefault(), new Consumer<Bitmap>()
        {
            @Override public void accept(Bitmap bitmap)
            {
                Utils.bitmapToMat(bitmap, currentCVFrame);
            }
        }));

        // faster method

//        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
//        {
//            @Override public void accept(Frame frame)
//            {
//                cameraFrame.getImageData();
//                Size size = cameraFrame.getSize();
//                int width = size.getWidth();
//                int height = size.getHeight();
//                ByteBuffer buffer = ByteBuffer.wrap(cameraFrame.getImageData());
//                Mat src = new Mat(height, width, CvType.CV_8UC2, buffer);
//                currentCVFrame = new Mat(height, width, CvType.CV_8UC3);
//                Imgproc.cvtColor(src, currentCVFrame, Imgproc.COLOR_YUV2RGB_YUY2);
//            }
//        }));
    }

    @Override
    protected void stop() {

    }
}
