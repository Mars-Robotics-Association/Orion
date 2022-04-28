package org.firstinspires.ftc.teamcode.Navigation.OpenCV;

import android.graphics.Bitmap;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import java.util.concurrent.BlockingQueue;

public class FrameAnaylisHelp implements VuforiaLocalizer {
    @Override
    public VuforiaTrackables loadTrackablesFromAsset(String assetName) {
        return null;
    }

    @Override
    public VuforiaTrackables loadTrackablesFromFile(String absoluteFileName) {
        return null;
    }

    @Nullable
    @Override
    public Camera getCamera() {
        return null;
    }

    @NonNull
    @Override
    public CameraName getCameraName() {
        return null;
    }

    @Override
    public CameraCalibration getCameraCalibration() {
        return null;
    }

    @Override
    public BlockingQueue<CloseableFrame> getFrameQueue() {
        return null;
    }

    @Override
    public void setFrameQueueCapacity(int capacity) {

    }

    @Override
    public int getFrameQueueCapacity() {
        return 0;
    }

    @Override
    public void getFrameOnce(Continuation<? extends Consumer<Frame>> frameConsumer) {

    }

    @Override
    public void close() {

    }

    @Override
    public void enableConvertFrameToBitmap() {

    }

    @Override
    public boolean[] enableConvertFrameToFormat(int... pixelFormats) {
        return new boolean[0];
    }

    @Nullable
    @Override
    public Bitmap convertFrameToBitmap(Frame frame) {
        return null;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

    }

}
