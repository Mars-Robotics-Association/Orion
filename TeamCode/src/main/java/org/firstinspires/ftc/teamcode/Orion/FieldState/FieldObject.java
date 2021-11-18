package org.firstinspires.ftc.teamcode.Orion.FieldState;

//used to store information about the position of objects on the field
public class FieldObject
{
    public String id;
    public String category;
    public enum ObjectType {
        STATIC,//object never moves, state should never be deleted
        SEMISTATIC,//object rarely moves and always exists, state should persist unless changed
        DYNAMIC,//object will move, state should re-assess whenever necessary (eg. camera points at it)
        PROBABILITY//object might exist, only on list when detected, deleted whenever not seen
    }
    public ObjectType objectType;
    public double[] poseToRobot = {0,0,0};
    public double[] poseToGlobal = {0,0,0};
    public double poseToRobotCertainty = 1;
    public double boundsRadius = 1;
    public int timeUpdated = 0;
}
