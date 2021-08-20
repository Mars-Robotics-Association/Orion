package org.firstinspires.ftc.teamcode.Orion.NavProfiles;

//Config for anything to do with vision / the webcam(s)
public abstract class VisionProfile
{
    //TF Object Detector
    public abstract String TFOD_MODEL_ASSET () ;
    public abstract String LABEL_FIRST_ELEMENT () ;
    public abstract String LABEL_SECOND_ELEMENT () ;
    public abstract double cameraXOffset () ;

    public VisionProfile(){}
}
