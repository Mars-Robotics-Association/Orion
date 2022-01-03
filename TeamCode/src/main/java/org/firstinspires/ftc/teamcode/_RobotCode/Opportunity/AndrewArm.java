package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AndrewArm {

    private DcMotor armMotor;
    private int armStartPos = 0;
    private double target = 0;
    private double armAngle;
    private double angleDiff;
    private double speed;
    private double rawPower;
    private double adjustmentSpeed;

    public int targetCount = 0;

    private boolean targetDirection = false;

    private static double zeroButtonOffset = 0.2; //this is kinda arbitrary


    public AndrewArm(DcMotor armPos){
        this.armMotor = armPos;
        armStartPos = armPos.getCurrentPosition();
    }
    public void zeroArm(){
        armStartPos = armMotor.getCurrentPosition();
    }

    public void loop(){
        armAngle = getAngle();
        double newPower = 0;

        if(target>-99){
            if(target>armAngle&&targetDirection)newPower = speed;     //could make this ease in more
            if(target<armAngle && !targetDirection)newPower = 0-speed;
        }
        if(Math.abs(target-armAngle)<angleDiff)newPower=0;

        if(rawPower!=0){
            newPower = rawPower;
        }

        armMotor.setPower(newPower+adjustmentSpeed);
    }

    public void setRawPower(double rawPower){
        target=-99;
        this.rawPower = rawPower;
        targetCount++;
    }

    public void setTarget(double angle,double speed,double angleDiff){
        targetCount++;
        if(Math.abs(angle-getAngle())<angleDiff) return;
        targetDirection = target>armAngle;
        rawPower = 0;
    this.angleDiff = angleDiff;
    this.speed = speed;
    target = angle;
    armMotor.setPower(speed);
    if(speed>0&&angle>armAngle){

    }
    }

    public double getTarget(){
        return target;
    }
    public double getAngle(){
        return (armMotor.getCurrentPosition()-armStartPos)/6000.0-zeroButtonOffset;
    }

    public void setAdjustmentSpeed(double adjustmentSpeed){
        this.adjustmentSpeed = adjustmentSpeed;
    }

}
