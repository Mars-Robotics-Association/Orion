package org.firstinspires.ftc.teamcode.Navigation.FieldState;

public class Pose
{
    public double X;
    public double Y;
    public double H;

    public Pose (double x, double y, double h){
        X = x;
        Y = y;
        H = h;
    }

    public void SetPose (double x, double y, double h){
        X = x;
        Y = y;
        H = h;
    }
}
