package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;

public class AndrewIMU {
    IMU imu;
    private double lastAngle;
    private double absoluteAngle;
    public AndrewIMU(IMU initializedIMU){
        imu = initializedIMU;
        absoluteAngle = 0;
    }
    public void loop(){
    double thisAngle = imu.GetRobotAngle();
    double adjustedAngle = thisAngle+180;

    if(adjustedAngle<100&&lastAngle>260){
        absoluteAngle +=adjustedAngle;
    }else if(adjustedAngle>260&&lastAngle<100){
        absoluteAngle+= 360-adjustedAngle;
    }else{
        absoluteAngle+= thisAngle-lastAngle; //maybe reverse that
    }


    lastAngle = adjustedAngle;
    }
    public double getRotation(){
       return absoluteAngle;
    }
    public void resetRotation(){
        absoluteAngle = 0;
    }

}
