/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.Navigation.OpenCV;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class VuMarkReader {
    private VuforiaTrackable relicTemplate;
    private VuforiaLocalizer vuforia;
    Image rgb;
    public double distance; //Distance from phone to cryptobox, updated by detectCryptobox() method

    static {
        System.loadLibrary("opencv_java3");
    }


    public VuMarkReader(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AeZ+Eyv/////AAABmfcFKgZ5NkXfgqEeyUnIJMIHuzBJhNTY+sbZO+ChF7mbo1evegC5fObZ830PRBTGTM6jbp+\" +\n" +
                "                    \"1XXCzx3XhY1kaZevEQXNpAKhXU9We0AMlp1mhnAUjGI2sprJZqJIfFGxkK598u8Bj3qQy4+PlCrk+Od\" +\n" +
                "                    \"/tAGs8dqAAsZPp4KpczFQttxMBC5JZNeIbIFP57InXOeJgyeH1sXK+R2i6nPfOFRvHJjdQaLWdAasv7\" +\n" +
                "                    \"i3b0RH5ctG7Ky7J9g9BPYI03kkChCJkbPg03XnoqCcC7rEpAk3n8a9CqtwTUu57Sy0jCDUd2O6X9kHj\" +\n" +
                "                    \"Z5ZmS0I3O0YSzX3Jp2ppTE2kDS2I9zBYEmuEqkMjItxd52oES0Ij0rZm";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
    }

    public RelicRecoveryVuMark getVuMark() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    public String detectJewel() {
        VuforiaLocalizer.CloseableFrame frame = null;
        String orientation = "";

        try {
            frame = vuforia.getFrameQueue().take();

            for (int i = 0; i < frame.getNumImages(); i++) {
                Image img = frame.getImage(i);

                if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);

                    Bitmap bitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bitmap.copyPixelsFromBuffer(rgb.getPixels());

                    //Determine orientation of jewels
                    Mat mat = new Mat();
                    Mat rotated = new Mat();
                    Mat processed;
                    Mat blue = new Mat();
                    Mat red = new Mat();
                    Mat red1 = new Mat();
                    Mat red2 = new Mat();

                    Utils.bitmapToMat(bitmap, mat);
                    Core.rotate(mat, rotated, Core.ROTATE_90_COUNTERCLOCKWISE);

                    //Crop image to bottom right corner
                    //New image has width of 200px and height of 400px
                    processed = new Mat(rotated, new Rect(rotated.width() - 200, rotated.height() -  300, 150, 200));

                    Imgproc.blur(processed, processed, new Size(10, 10));
                    Imgproc.cvtColor(processed, processed, Imgproc.COLOR_RGB2HSV);

                    Core.inRange(processed, new Scalar(84, 177, 89), new Scalar(126, 255, 255), blue); //Detect blue HSV values

                    Core.inRange(processed, new Scalar(0, 117, 115), new Scalar(6, 255, 255), red1); //Detect red HSV values
                    Core.inRange(processed, new Scalar(114, 209, 106), new Scalar(180, 255, 230), red2);
                    Core.bitwise_or(red1, red2, red); //Combine upper and lower red hue ranges

                    Imgproc.erode(blue, blue, new Mat(), new Point(-1, -1), 5, Core.BORDER_CONSTANT, new Scalar(-1));
                    Imgproc.dilate(blue, blue, new Mat(), new Point(-1, -1), 25, Core.BORDER_CONSTANT, new Scalar(-1));

                    Imgproc.erode(red, red, new Mat(), new Point(-1, -1), 5, Core.BORDER_CONSTANT, new Scalar(-1));
                    Imgproc.dilate(red, red, new Mat(), new Point(-1, -1), 25, Core.BORDER_CONSTANT, new Scalar(-1));

                    List<MatOfPoint> blueContours = new ArrayList<>();
                    List<MatOfPoint> redContours = new ArrayList<>();
                    Mat hierarchy = new Mat();

                    Imgproc.findContours(blue, blueContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                    Imgproc.findContours(red, redContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    //Determine orientation of jewel
                    if(blueContours.size() == 1 && redContours.size() == 0) orientation = "Blue"; //Red jewel is on right
                    else if (blueContours.size() == 0 && redContours.size() == 1) orientation = "Red";
                    else orientation = "Unknown";

                    mat.release();
                    rotated.release();
                    processed.release();
                    blue.release();
                    red.release();
                    red1.release();
                    red2.release();

                    break;
                }
            }
        } catch(InterruptedException e) {
            e.printStackTrace();
            orientation = "Unknown";
        } finally {
            if (frame != null) frame.close();
        }

        return orientation;
    }

    public ArrayList<Integer> detectCryptobox(String allianceColor) {
        VuforiaLocalizer.CloseableFrame frame = null;
        ArrayList<Integer> xCoordinates = new ArrayList<>(); //Contains coordinates of center of each detected column

        try {
            frame = vuforia.getFrameQueue().take();

            for (int i = 0; i < frame.getNumImages(); i++) {
                Image img = frame.getImage(i);

                if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);

                    Bitmap bitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bitmap.copyPixelsFromBuffer(rgb.getPixels());

                    Mat mat = new Mat();
                    Mat rotated = new Mat();
                    Mat processed = new Mat();
                    Mat red1 = new Mat();
                    Mat red2 = new Mat();

                    Utils.bitmapToMat(bitmap, mat);
                    Core.rotate(mat, rotated, Core.ROTATE_90_COUNTERCLOCKWISE);
                    Imgproc.blur(rotated, processed, new Size(10, 10));
                    Imgproc.cvtColor(processed, processed, Imgproc.COLOR_RGB2HSV);

                    if(allianceColor.equals("Blue")) {
                        Core.inRange(processed, new Scalar(67, 80, 40), new Scalar(135, 255, 137), processed);
                    } else {
                        Core.inRange(processed, new Scalar(0, 117, 115), new Scalar(6, 255, 255), red1);
                        Core.inRange(processed, new Scalar(114, 209, 106), new Scalar(180, 255, 230), red2);
                        Core.bitwise_or(red1, red2, processed);
                    }

                    Imgproc.erode(processed, processed, new Mat(), new Point(-1, -1), 8, Core.BORDER_CONSTANT, new Scalar(-1));
                    Imgproc.dilate(processed, processed, new Mat(), new Point(-1, -1), 10, Core.BORDER_CONSTANT, new Scalar(-1));

                    List<MatOfPoint> contours = new ArrayList<>();
                    Mat hierarchy = new Mat();
                    Imgproc.findContours(processed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    //Combine contours
                    ArrayList<Rect> boundingRects = new ArrayList<>();

                    for(i = 0; i < contours.size(); i++) {
                        Rect rect = Imgproc.boundingRect(contours.get(i));

                        //Remove contour if it is roughly square - ie. jewel
                        //All detected contours should have a height that is larger than the width - vertical rectangle
                        if((double) rect.height / (double) rect.width > 1.4) boundingRects.add(rect);
                    }

                    ArrayList<Rect> columns = new ArrayList<>();

                    for(i = 0; i < boundingRects.size(); i++) {
                        //Only add to list if it is a unique X coordinate
                        boolean addToList = true;

                        //Remove contours if they have similar X values
                        //Multiple contours for the same column are detected because they are separated by the white tape line
                        for(int j = 0; j < columns.size(); j++) {
                            if(Math.abs(boundingRects.get(i).x - columns.get(j).x) < 100) {
                                addToList = false;
                                break;
                            }
                        }

                        if(addToList) {
                            columns.add(boundingRects.get(i));
                        }
                    }

                    //Columns now contains a list of the cryptobox columns

                    //Sort columns to determine distance between adjacent columns
                    Collections.sort(columns, new Comparator<Rect>() {
                        @Override
                        public int compare(Rect lhs, Rect rhs) {
                            return lhs.x > rhs.x ? 1 : -1;
                        }
                    });

                    //Determine position of center of each column, ignoring last column
                    //For loop will not execute if only one column is detected
                    for(i = 0; i < columns.size() - 1; i++) {
                        //X values represent the left side of the bounding box
                        int x = columns.get(i).x + columns.get(i).width + ((columns.get(i + 1).x - (columns.get(i).x + columns.get(i).width)) / 2);
                        xCoordinates.add(x);
                    }

                    double avg = 0;

                    for(i = 0; i < columns.size() - 1; i++) {
                        avg += columns.get(i + 1).x - columns.get(i).x;
                    }

                    avg /= (double) (columns.size() - 1);

                    //Distance is based on average width between cryptobox columns
                    //Based upon regression of reciprocal function
                    distance = 9090.64 / (avg + 29.9988) + -1.6938;

                    //Release Mat objects to avoid Heap size / out of memory exception
                    mat.release();
                    rotated.release();
                    processed.release();
                    red1.release();
                    red2.release();

                    break;
                }
            }
        } catch(InterruptedException e) {
            e.printStackTrace();
        } finally {
            if (frame != null) frame.close();
        }

        return xCoordinates;
    }
}