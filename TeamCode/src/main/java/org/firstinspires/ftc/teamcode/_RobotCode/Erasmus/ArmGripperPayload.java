package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;

class ArmGripperPayload
{
    EncoderActuator arm;
    Servo gripper;

    boolean gripperClosed = false;

    public ArmGripperPayload(EncoderActuator setArm, Servo setGripper){
        arm = setArm;
        gripper = setGripper;
    }

    public void setGripper(boolean closed){
        if(closed) gripper.setPosition(0);
        else gripper.setPosition(0.5);
        gripperClosed = closed;
    }
    public void toggleGripper(){
        if(gripperClosed) setGripper(false);
        else setGripper(true);
    }

    public void moveArm(double speed){arm.setPowerClamped(speed);}
    public void setArmDegrees(double degrees){arm.goToPosition(degrees);}
    public void armToMax(){arm.goToMax();}
    public void armToMin(){arm.goToMin();}
}
