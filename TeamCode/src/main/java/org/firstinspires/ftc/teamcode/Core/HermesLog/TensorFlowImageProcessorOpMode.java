package org.firstinspires.ftc.teamcode.Core.HermesLog;

import com.acmerobotics.dashboard.config.Config;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.OpenCV.EncodedImageRecognition;
import org.firstinspires.ftc.teamcode.OpenCV.Pipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

@TeleOp(name="Tensor Flow Image Processor", group="Testing")
@Config
public class TensorFlowImageProcessorOpMode extends OpMode
{
    public static final String TAG = "TensorFlowImageProcessorOpMode";
    //RobotPose poseToSend;
    private WebcamName webcamName;
    private Pipeline pipline;

    private ExecutorService frameConsumerExecutor;

    private Bitmap latestImage; //use set/get methods, do not access directly!

    private synchronized void setLatestImage(Bitmap newbm) {
        latestImage = newbm;
    }

    private synchronized Bitmap getLatestImage() {
        return latestImage;
    }

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AeZ+Eyv/////AAABmfcFKgZ5NkXfgqEeyUnIJMIHuzBJhNTY+sbZO+ChF7mbo1evegC5fObZ830PRBTGTM6jbp+" +
                    "1XXCzx3XhY1kaZevEQXNpAKhXU9We0AMlp1mhnAUjGI2sprJZqJIfFGxkK598u8Bj3qQy4+PlCrk+Od" +
                    "/tAGs8dqAAsZPp4KpczFQttxMBC5JZNeIbIFP57InXOeJgyeH1sXK+R2i6nPfOFRvHJjdQaLWdAasv7" +
                    "i3b0RH5ctG7Ky7J9g9BPYI03kkChCJkbPg03XnoqCcC7rEpAk3n8a9CqtwTUu57Sy0jCDUd2O6X9kHj" +
                    "Z5ZmS0I3O0YSzX3Jp2ppTE2kDS2I9zBYEmuEqkMjItxd52oES0Ij0rZm";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Gson gson;
    private DashboardWebSocketServer server;


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    void initVuforia() {

        //Context context = AppUtil.getDefContext();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        params.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(params);

        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();

    }

    void initDashboard() {
        server = DashboardWebSocketServer.getInstance();
        if ( server == null) {
            server = new DashboardWebSocketServer();
            try {
                server.start(10000, false);
            } catch (IOException e) {
                Log.w("failed to start server", e);
            }
        }
    }

    @Override
    public void init() {
        initDashboard();

        gson = new GsonBuilder().create();

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            //tfod.setZoom(2.5, 16.0/9.0);
        }

        latestImage = null;
    }

    @Override
    public void start() {
        frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("Vuforia frame consumer");
        frameConsumerExecutor.execute(new FrameConsumer(vuforia.getFrameQueue()));
    }

    @Override
    public void stop() {
        if (tfod != null) {
            tfod.shutdown();
        }
        if (frameConsumerExecutor != null) {
            frameConsumerExecutor.shutdownNow();
            frameConsumerExecutor = null;
        }
    }

    private class FrameConsumer implements Runnable {
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;

        private FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
            this.frameQueue = frameQueue;
        }

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                if (!frameQueue.isEmpty()) {
                    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
                    try {
                        vuforiaFrame = frameQueue.take();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    if (vuforiaFrame == null) {
                        continue;
                    }

                    getImageFromFrame(vuforiaFrame);
                } else {
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }

        private void getImageFromFrame(VuforiaLocalizer.CloseableFrame frame) {
            long numImgs = frame.getNumImages();
            for (int i = 0; i < numImgs; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    Image img = frame.getImage(i);
                    if (img != null) {
                        Bitmap bitmap = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                        bitmap.copyPixelsFromBuffer(img.getPixels());
                        setLatestImage(bitmap);
                        frame.close();
                        return;
                    }
                }
            }
            frame.close();
        }
    }

    private void getRecognitions(Bitmap bitmap) {
        if (tfod != null && bitmap != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {

                    Mat mf = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC3);
                    Utils.bitmapToMat(bitmap, mf);
                    //Core.flip(mf, mf, 1);
                    //Core.rotate(mf, mf, Core.ROTATE_90_COUNTERCLOCKWISE);

                    int left = Math.round(recognition.getLeft());
                    if (left < 0) left = 0;

                    int top = Math.round(recognition.getTop());
                    if (top < 0) top = 0;

                    int right = Math.round(recognition.getRight());
                    if (right > mf.width()) right = mf.width();

                    int bottom = Math.round(recognition.getBottom());
                    if (bottom > mf.height()) bottom = mf.height();

                    int dx = right - left;
                    int dy = bottom - top;
                    int area = dx*dy;

                    Log.i(TAG, "image coords: " + left + ", " + top + ", " + dx + ", " + dy + "(" + right + ", " + bottom + ")");

                    Rect rectcrop = new Rect(left, top, dx, dy);

                    //opencv functions
                    Mat cropped = new Mat(mf, rectcrop);
                    Scalar lowbgr = new Scalar(0,100,100);
                    Scalar highbgr = new Scalar(30, 255, 255);
                    Mat hsv = new Mat();
                    Mat mask = new Mat();
                    Mat last = new Mat();
                    Imgproc.cvtColor(cropped, hsv, Imgproc.COLOR_RGB2HSV);
                    Core.inRange(hsv, lowbgr,highbgr, mask);
                    Core.bitwise_and(cropped, cropped, last, mask);
                    cropped = last;


                    Bitmap revised = Bitmap.createBitmap(cropped.width(), cropped.height(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(cropped, revised);

                    int pixelcount = 0;

                    for (int x = 0; x < dx; x++) {
                        for (int y = 0; y < dy; y++) {
                            double[] data = cropped.get(x, y); //Stores element in an array
                            for (int k = 0; k < 3; k++) //Runs for the available number of channels
                            {
                                data[k] = data[k] * 2; //Pixel modification done here
                            }
                            if((data[0] >100 && data[0]<255)&&(data[1] >100 && data[1]<255)&&(data[2] >0 && data[2]<30)){
                                pixelcount++;
                            }
                        }
                    }
                    float percent = pixelcount/area;
                    telemetry.addData("percent yellow: ",percent);

                    EncodedImageRecognition cr = new EncodedImageRecognition(revised);
                    String msg = gson.toJson(cr);
                    DashboardWebSocketServer.getInstance().send(msg);

                    telemetry.addData("left,top:  ", "%d, %d", left, top);
                    telemetry.addData("dx,dy:  ", "%d, %d", dx, dy);
                }
            }
        }
    }

    @Override
    public void loop() {
        getRecognitions(getLatestImage());
    }
}
