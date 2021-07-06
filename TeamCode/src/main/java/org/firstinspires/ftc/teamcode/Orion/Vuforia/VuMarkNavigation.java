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
package org.firstinspires.ftc.teamcode.Orion.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

public class VuMarkNavigation {

    public static final String TAG = "Vuforia VuMark Finder";
    private OpMode opMode;
    private static final float mmPerInch = 25.4f;

    OpenGLMatrix lastLocation = null;

    private VuforiaLocalizer vuforia;
    public VuforiaLocalizer GetVuforia() {return vuforia;}
    private VuforiaTrackables trackables;

    private WebcamName webcamName;

    public VuMarkNavigation(OpMode opmode, String webcamID) {
        opMode = opmode;

        webcamName = opmode.hardwareMap.get(WebcamName.class, webcamID);
        int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AeZ+Eyv/////AAABmfcFKgZ5NkXfgqEeyUnIJMIHuzBJhNTY+sbZO+ChF7mbo1evegC5fObZ830PRBTGTM6jbp+1XXCzx3XhY1kaZevEQXNpAKhXU9We0AMlp1mhnAUjGI2sprJZqJIfFGxkK598u8Bj3qQy4+PlCrk+Od/tAGs8dqAAsZPp4KpczFQttxMBC5JZNeIbIFP57InXOeJgyeH1sXK+R2i6nPfOFRvHJjdQaLWdAasv7i3b0RH5ctG7Ky7J9g9BPYI03kkChCJkbPg03XnoqCcC7rEpAk3n8a9CqtwTUu57Sy0jCDUd2O6X9kHjZ5ZmS0I3O0YSzX3Jp2ppTE2kDS2I9zBYEmuEqkMjItxd52oES0Ij0rZm";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        trackables = GetVuforia().loadTrackablesFromAsset("UltimateGoal");

        opMode.telemetry.update();

        trackables.activate();
    }

    public double[] GetData(int vumarkIndex) {

        double[] data = {0.0,0.0,0.0,0.0,0.0,0.0};
        VuforiaTrackable vumark = trackables.get(vumarkIndex);

        VuforiaTrackable vuMark = vumark;
        if (vuMark != null) {

            /* Found an instance of the template. In the actual game, you will probably
             * loop until this condition occurs, then move on to act accordingly depending
             * on which VuMark was visible. */
            opMode.telemetry.addData("VuMark", "%s visible", vuMark.getName());

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)vumark.getListener()).getFtcCameraFromTarget();
            opMode.telemetry.addData("Pose", format(pose));

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

                double dist = Math.sqrt(Math.pow(Math.abs(tX),2)+Math.pow(Math.abs(tY),2));//tX use to by tY, this was switched because the camera uses portrait mode
                double rZreal = Math.toDegrees(Math.atan(tX/tZ));
                //opMode.telemetry.addData("Vumark",dist + " milimeters away");
                //opMode.telemetry.addData("Vumark",-1*tX+" milimeters high");
                data[0] = tX / mmPerInch;
                data[1]= tY / mmPerInch;
                data[2] = tZ / mmPerInch;
                data[3] = dist / mmPerInch;
                data[4] = rZreal;
                data[5] = rY;
            }

        }
        else {
            opMode.telemetry.addData("VuMark", "not visible");
        }
        return data;

    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
