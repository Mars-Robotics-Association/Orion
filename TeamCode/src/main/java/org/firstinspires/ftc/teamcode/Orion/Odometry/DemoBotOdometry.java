package org.firstinspires.ftc.teamcode.Orion.Odometry;

//For getting input back from the odometry unit and running calculations based on it.

//REQUIRED TO RUN: Phones | REV Hub | Odometry Unit


import com.qualcomm.robotcore.hardware.DcMotor;

public class DemoBotOdometry
{
    ////Dependencies////
    //Old last year method
    private DcMotor EncoderX;
    private DcMotor EncoderY;

    ////Variables////
    private double X;
    private double Y;
    private double TargetDist;
    private double TicksPerInchX = 88; //how many encoder ticks per one inch
    private double TicksPerInchY = 88; //how many encoder ticks per one inch

    //Initializer
    public DemoBotOdometry(DcMotor encoderX, DcMotor encoderY){
        EncoderX = encoderX;
        EncoderY = encoderY;
    }

    ////STARTUP////

    ////CALLABLE METHODS////
    public void Reset(){
        //Set x and y to 0
        X = 0;
        Y = 0;
        //reset encoders
        EncoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EncoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void SetTargetDist(double dist) {
        //Sets the target X and Y for odometry
        TargetDist = dist;
    }
    public boolean IsAtTargetDist(double threshold){
        //Determines whether the robot is close enough to the target distance
        if(Math.abs(TargetDist - CalculateDistance()) < threshold){return true;}
        else {return false;}
    }
    public void SetEncoderValsDirect(double x, double y){
        //sets x and y to input
        X = x;
        Y = y;
    }
    public void SetEncoderVals(){
        X = GetEncoderVals()[0];
        Y = GetEncoderVals()[1];
    }
    public double[] GetEncoderVals(){
        //returns the current positions of the encoders on the robot
        double[] vals = {EncoderX.getCurrentPosition(), EncoderY.getCurrentPosition()};
        return vals;
    }
    public double CalculateDistance(){
        //Gets input
        SetEncoderVals();
        //Get distance in each direction in inches
        double distX = X/TicksPerInchX;
        double distY = Y/TicksPerInchY;
        //Calculates distance robot has gone using pythagorean theorem. Returns the distance in inches
        return Math.sqrt((distX*distX) + (distY*distY));
    }

    ////PRIVATE METHODS////

}
