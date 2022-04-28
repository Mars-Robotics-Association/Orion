package org.firstinspires.ftc.teamcode.Navigation.FieldState;

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
    public Pose poseToRobot;
    public Pose poseToGlobal;
    public double poseCertainty = 1;
    public double boundsRadius = 1;
    public int timeUpdated = 0;

    public FieldObject(String set_id, String set_category, ObjectType set_objectType, Pose set_poseToRobot, Pose set_poseToGlobal, double set_poseCertainty, double set_boundsRadius){
        id = set_id;
        category = set_category;
        objectType = set_objectType;
        poseToRobot = set_poseToRobot;
        poseToGlobal = set_poseToGlobal;
        poseCertainty = set_poseCertainty;
        boundsRadius = set_boundsRadius;
    }
}
