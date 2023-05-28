package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import static com.sun.tools.javac.util.Assert.error;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

import java.util.concurrent.TimeUnit;

/**
 * Part of the robot head that manages the camera feed.
 * Data is sent in raw YUNV/YUV2 format to reduce necessary data transfer.
 *
 * @deprecated The Bluetooth camera makes this Behavior useless.
 */
public class HeadCamera extends Behavior {
    private static final String TAG = "Head Camera";

    CameraCaptureSession cameraCaptureSession;

    void startCamera() throws Exception {
        Handler callbackHandler = CallbackLooper.getDefault().getHandler();

        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        final int imageFormat = ImageFormat.YUY2;

        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        Deadline deadline = new Deadline(Integer.MAX_VALUE, TimeUnit.SECONDS);
        Camera camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            throw new Exception("No camera found");
        }

        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, 5);
                        session.startCapture(captureRequest,
                                (session1, request, cameraFrame) -> {
                                    byte[] bytes = cameraFrame.getImageData();
                                },
                                Continuation.create(callbackHandler, (session12, cameraCaptureSequenceId, lastFrameNumber) -> RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber))
                        );
                        synchronizer.finish(session);
                    } catch (CameraException | RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            error("exception starting camera");
            synchronizer.finish(null);
        }

        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        cameraCaptureSession = synchronizer.getValue();
    }

//    private Thread streamingServer = new Thread(new Runnable() {
//        private DatagramSocket socket;
//        private boolean running;
//
//        private byte[] imageToSend = new byte[]{};
//
//        private byte[] handshake = new byte[256];
//
//        private InetAddress receiverAddr = null;
//        private int receiverPort = 0;
//
//        @Override
//        public void run() {
//            while (true) {
//                try {
//
//                }catch(Exception e){
//                    RobotLog.e(e.toString());
//                }
//            }
//        }
//    });

    /**
     * {@inheritDoc}
     */
    @Override
    protected void init() throws Exception {

    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void start() {

    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void update() {

    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void stop() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }
}
