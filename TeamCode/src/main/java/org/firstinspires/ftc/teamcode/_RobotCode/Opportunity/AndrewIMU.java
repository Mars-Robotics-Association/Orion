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

    if(thisAngle<-100&&lastAngle>100){
        absoluteAngle+= Math.abs(thisAngle%180);
    }else if(thisAngle>100&&lastAngle<-100){
        absoluteAngle-=Math.abs(thisAngle%180);
    }else{
        absoluteAngle+=thisAngle-lastAngle;
    }

    lastAngle = thisAngle;
    }
    public double getRotation(){
       return absoluteAngle;
    }
    public void resetRotation(){
        absoluteAngle = 0;
    }

}
