package org.firstinspires.ftc.teamcode.Orion.NavModules;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DashboardWebSocketServer;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.Base64Image;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.ConfidenceLevel;
import org.firstinspires.ftc.teamcode.Orion.NavModules.OpenCV.EncodedImageRecognition;
import org.firstinspires.ftc.teamcode.Orion.NavModules.OpenCV.Pipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.Utils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;

public class Camera
{
    private OpMode opmode;
    private WebcamName webcamname;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private TFObjectDetector tfod;
    private EvictingBlockingQueue<Bitmap> frameQueue;
    private FtcDashboard dashboard;

    public VuforiaLocalizer GetVuforia() {return vuforia;}

    private int cameraMonitorViewID;
    private static final String VLK = "AeZ+Eyv/////AAABmfcFKgZ5NkXfgqEeyUnIJMIHuzBJhNTY+sbZO+ChF7mbo1evegC5fO" +
            "bZ830PRBTGTM6jbp+1XXCzx3XhY1kaZevEQXNpAKhXU9We0AMlp1mhnAUjGI2sprJZqJIfFGxkK598u8Bj3qQy4+P" +
            "lCrk+Od/tAGs8dqAAsZPp4KpczFQttxMBC5JZNeIbIFP57InXOeJgyeH1sXK+R2i6nPfOFRvHJjdQaLWdAasv7i3" +
            "b0RH5ctG7Ky7J9g9BPYI03kkChCJkbPg03XnoqCcC7rEpAk3n8a9CqtwTUu57Sy0jCDUd2O6X9kHjZ5ZmS0I" +
            "3O0YSzX3Jp2ppTE2kDS2I9zBYEmuEqkMjItxd52oES0Ij0rZm";


    public Camera(OpMode opMode, String webCam) {
        opmode = opMode;

        webcamname = opmode.hardwareMap.get(WebcamName.class,webCam);
        cameraMonitorViewID = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewID);

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamname, cameraMonitorViewID);
        camera.openCameraDevice();
        camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(new Pipeline());

        parameters.vuforiaLicenseKey = VLK;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = webcamname;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        trackables = GetVuforia().loadTrackablesFromAsset("UltimateGoal");

        opMode.telemetry.update();

        trackables.activate();
        initializeFrameQueue(2);

        //Init
        initTfod();
        if (tfod != null)tfod.activate();

        dashboard = FtcDashboard.getInstance();

        dashboard.startCameraStream(vuforia,0);
    }

    private void initTfod() {
        int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("UltimateGoal.tflite", "Ball", "Cube");
        tfod.setZoom(1, 16.0/9.0);
    }

    public void stopTfod(){
        if (tfod != null) tfod.shutdown();
    }

    //Gets a list of Tensorflow recognitions
    public List<Recognition> GetRecognitions() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getRecognitions();
            if (recognitions != null) {
                opmode.telemetry.addData("# Object Detected", recognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : recognitions) {
                    opmode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    opmode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    opmode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            }
            return recognitions;
        }
        else return null;
    }
    //Prints data for TensorFlow detections
    public void PrintTFTelemetry(){
        List<Recognition> items = GetRecognitions();
        int amount = 0;
        if(items != null){
            for (Recognition r : items) {
                opmode.telemetry.addLine("Top: "+r.getTop()+" | Bottom: "+r.getBottom()+" | Left: "+r.getLeft()+"| Right: "+r.getRight());
            }
        }
    }

    //returns an array of values for the position of a bounding box
    public double[] GetPoseToCamera(int vumarkIndex) {

        double[] data = {0.0,0.0,0.0,0.0,0.0,0.0};
        VuforiaTrackable vumark = trackables.get(vumarkIndex);

        VuforiaTrackable vuMark = vumark;
        if (vuMark != null) {

            /* Found an instance of the template. In the actual game, you will probably
             * loop until this condition occurs, then move on to act accordingly depending
             * on which VuMark was visible. */
            opmode.telemetry.addData("VuMark", "%s visible", vuMark.getName());

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)vumark.getListener()).getFtcCameraFromTarget();
            opmode.telemetry.addData("Pose", format(pose));

            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

                double dist = Math.sqrt(Math.pow(Math.abs(tX),2)+Math.pow(Math.abs(tY),2));//tX use to be tY, this was switched because the camera uses portrait mode
                double rZreal = Math.toDegrees(Math.atan(tX/tZ));
                //opMode.telemetry.addData("Vumark",dist + " milimeters away");
                //opMode.telemetry.addData("Vumark",-1*tX+" milimeters high");
                data[0] = tX / 25.4f;
                data[1]= tY / 25.4f;
                data[2] = tZ / 25.4f;
                data[3] = dist / 25.4f;
                data[4] = rZreal;
                data[5] = rY;
            }

        }
        else {
            opmode.telemetry.addData("VuMark", "not visible");
        }
        return data;

    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    //returns array of values for closed TensorFlow bounding box by y value
    public double[] GetClosestFrieght(){ //Find object by its y distance from the robot
        List<Recognition> objs = GetRecognitions(); //find all objects
        double x = 0;
        double y = 0;
        double width = 0;
        if(objs != null) {
            for (Recognition obj : objs) {
                if (obj.getLabel() == "Cube" || obj.getLabel() == "Ball") { //accept all freight
                    //Find x and y of disc
                    double discX = (obj.getRight() + obj.getLeft()) / 2;
                    discX += -400;
                    double discY = (obj.getTop() + obj.getBottom()) / 2;
                    double discWidth = obj.getWidth();
                    //if current dist is less than highest dist or if its the first loop
                    if(discY < y || y==0){
                        //set lowest values as these
                        x = discX;
                        y = discY;
                        width = discWidth;
                    }
                }
            }
        }
        return new double[] {x,y,width};
    }

    //takes TensorFlow recognitions and runs them through OpenCV
    private void getRecognitions(Bitmap bitmap) {
        if (tfod != null && bitmap != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            EncodedImageRecognition bt = new EncodedImageRecognition(bitmap);
//            String bit = gson.toJson(bt);
//            FullIMG img = new FullIMG(bit);
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

                    double pixelcount = 0.0;

                    for (int x = 0; x <revised.getWidth(); x++) {
                        for (int y = 0; y < revised.getHeight(); y++) {
                            int color = revised.getPixel(x,y);
                            int R = (color & 0xff0000) >> 16;
                            int G = (color & 0xff00) >> 8;
                            int B = color & 0xff;
                            if((R != 0)&&(G != 0)&&(B != 0)){
                                pixelcount++;
                            }
                        }
                    }
                    double percent = 100.0* (pixelcount/area);
                    opmode.telemetry.addData("percent yellow: ",percent);
                    float vconf = recognition.getConfidence();
                    boolean iscube = false;
                    if (vconf>=.5 && percent >= 50){
                        iscube = true;
                    }
                    opmode.telemetry.addData("Recognition: ",iscube);

                    opmode.telemetry.addData("left,top:  ", "%d, %d", left, top);
                    opmode.telemetry.addData("dx,dy:  ", "%d, %d", dx, dy);
                    opmode.telemetry.update();
                }
            }
        }
    }

    //takes a Mat image and converts it to a Bitmap
    public Bitmap convertMatToBitMap(Mat input){
        Bitmap bmp = null;
        Utils.matToBitmap(input,bmp);
        return bmp;
    }

    public Mat convertBitmapToMat(Bitmap input){
        Mat mat = new Mat();
        Utils.bitmapToMat(input,mat);
        return mat;
    }

    //takes a Mat and isolates the color yellow
    public Mat IsolateYellow(Mat input){
        Scalar lowbgr = new Scalar(0,100,100);
        Scalar highbgr = new Scalar(30, 255, 255);
        Mat result;
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowbgr,highbgr, mask);
        Core.bitwise_and(input, input, last, mask);
        result = last;
        return result;
    }

    //takes a Mat and isolates the color white
    public Mat IsolateWhite(Mat input){
        Scalar highbgr = new Scalar(255,255,255);
        Scalar lowbgr = new Scalar(230,230,230);
        Mat result;
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowbgr,highbgr, mask);
        Core.bitwise_and(input, input, last, mask);
        result = last;
        return result;
    }

    //for determining nonwhite pixels in a cropped image
    public int countPixels(Bitmap input){
        int pixelcount = 0;

        for (int x = 0; x <input.getWidth(); x++) {
            for (int y = 0; y < input.getHeight(); y++) {
                int color = input.getPixel(x,y);
                int R = (color & 0xff0000) >> 16;
                int G = (color & 0xff00) >> 8;
                int B = color & 0xff;
                if((R != 0)&&(G != 0)&&(B != 0)){
                    pixelcount++;
                }
            }
        }
        return pixelcount;
    }

    //isolate a color from a mat
    public Mat isolateColor(Mat input, Scalar highbgr, Scalar lowbgr){
        Mat result;
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowbgr,highbgr, mask);
        Core.bitwise_and(input, input, last, mask);
        result = last;
        return result;
    }

    public Bitmap GetImage() throws InterruptedException {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
        vuforia.setFrameQueueCapacity(1);
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
//        Image rgb = null;
//        long numImages = frame.getNumImages();
//        for(int i=0;i<numImages;i++){
//            if(frame.getImage(i).getFormat()==PIXEL_FORMAT.RGB565){
//                rgb = frame.getImage(i);
//                break;
//            }
//        }
       Bitmap bmp = vuforia.convertFrameToBitmap(frame);

        frame.close();

        //Bitmap bmp = frameQueue.poll();
        return bmp;
    }


    private void initializeFrameQueue(int capacity) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }
}
