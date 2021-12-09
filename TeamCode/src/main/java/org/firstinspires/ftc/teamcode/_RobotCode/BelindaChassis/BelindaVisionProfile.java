package org.firstinspires.ftc.teamcode._RobotCode.BelindaChassis;

import org.firstinspires.ftc.teamcode.Orion.Archive.NavProfiles.VisionProfile;

public class BelindaVisionProfile extends VisionProfile
{
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static double cameraXOffset = -400.0;

    @Override
    public String TFOD_MODEL_ASSET() {return TFOD_MODEL_ASSET;}

    @Override
    public String LABEL_FIRST_ELEMENT() {return LABEL_FIRST_ELEMENT;}

    @Override
    public String LABEL_SECOND_ELEMENT() {return LABEL_SECOND_ELEMENT;}

    @Override
    public double cameraXOffset() {return cameraXOffset;}
}
