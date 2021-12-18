package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;

public class AndrewIMU {
    IMU imu;
    double lastAngle;
    public AndrewIMU(IMU initializedIMU){
        imu = initializedIMU;
    }
    public void loop(){
    imu.GetRobotAngle();
    }
    public double absoluteRotation(){
       return 0;
    }

}
