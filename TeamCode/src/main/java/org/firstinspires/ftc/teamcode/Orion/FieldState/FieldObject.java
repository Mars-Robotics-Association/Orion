package org.firstinspires.ftc.teamcode.Orion.FieldState;

public class FieldObject
{
    public String id;
    public String category;
    public enum ObjectType {
        STATIC,//object never moves, state should never be deleted
        SEMISTATIC,//object rarely moves, state should persist unless changed
        DYNAMIC,//object will move, state should re-assess whenever necessary (eg. camera points at it)
        PROBABILITY//object might exist, only on list when detected, deleted whenever not seen
    }
    public double[] poseToRobot = {0,0,0};
    public double[] poseToGlobal = {0,0,0};
    public double poseToRobotCertainty = 1;

}
