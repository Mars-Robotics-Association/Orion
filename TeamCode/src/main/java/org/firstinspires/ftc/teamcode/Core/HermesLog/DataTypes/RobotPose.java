package org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes;

public class RobotPose{
    public RobotPose(double target_x, double target_y, double target_h, double actual_x, double actual_y, double actual_h) {
        tx = target_x;
        ty = target_y;
        th = target_h;
        ax = actual_x;
        ay = actual_y;
        ah = actual_h;
    }
    public double tx;
    public double ty;
    public double th;
    public double ax;
    public double ay;
    public double ah;
}
