package org.firstinspires.ftc.teamcode.Orion;

class RobotTransformSystem
{
    ////Current Robot Coords////
    private double robotX;
    public double getRobotX() { return robotX; }
    private double robotY;
    public double getRobotY() { return robotY; }
    private double robotHeading;
    public double getRobotHeading() { return robotHeading; }

    public RobotTransformSystem(double startX, double startY, double startHeading){
        SetRobotGlobalPose(startX, startY, startHeading);
    }

    public void SetRobotGlobalPose(double x, double y, double heading){
        robotX = x;
        robotY = y;
        robotHeading = heading;
    }

    public double GetDeltaAngleToObject(double oX, double oY){
        //Calculate the distance robot needs to turn to face object
        double deltaX = oX-robotX;
        double deltaY = oY-robotY;
        double headingFromRobotCenter = Math.toDegrees(Math.atan2(deltaX, deltaY)) + 90;//TODO: make sure 90 degrees is the right offset!
        return headingFromRobotCenter - getRobotHeading();
    }

    public double[] ConvertToGlobalSimple(double x, double y, double heading){
        //Converts local (relative to robot) coords to global coords WITHOUT taking robot heading into account
        double offsetX = x + getRobotX();
        double offsetY = y + getRobotY();
        double offsetH = heading + getRobotHeading();
        double[] data = {offsetX, offsetY, offsetH};
        return data;
    }

    public double[] ConvertToGlobalComplex(double x, double y, double heading){//TODO: get the math figured out and fill this in!
        ////Converts local (relative to robot) coords to global coords taking robot heading into account
        //Math
        double robotHeadingRadians = Math.toRadians(robotHeading);
        double globalX = x * Math.cos(robotHeadingRadians) - y * Math.sin(robotHeadingRadians) + robotX;
        double globalY = x * Math.sin(robotHeadingRadians) + y * Math.cos(robotHeadingRadians) + robotY;
        double globalH = robotHeading + heading;
        //Return values
        double[] data = {globalX, globalY, globalH};
        return data;
    }

    public double[] ConvertToLocal(double x, double y, double heading){
        //Converts global coords to local (relative to robot) coords
        double offsetX = x - getRobotX();
        double offsetY = y - getRobotY();
        double offsetH = heading - getRobotHeading();
        double[] data = {offsetX, offsetY, offsetH};
        return data;
    }
}
