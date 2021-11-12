package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


class LiftController {
    public LiftController(int lowerBounds_e, int upperBounds_e, double speed_e, boolean raiseBoundErrors_e) {
        lowerBounds = lowerBounds_e;
        upperBounds = upperBounds_e;
        speed = speed_e;
        raiseBoundErrors = raiseBoundErrors_e;
    }

    public LiftController(int lowerBounds_e, int upperBounds_e, boolean raiseBoundErrors_e) {
        lowerBounds = lowerBounds_e;
        upperBounds = upperBounds_e;
        speed = 100;
        raiseBoundErrors = raiseBoundErrors_e;
    }

    public LiftController(int lowerBounds_e, int upperBounds_e, double speed_e) {
        lowerBounds = lowerBounds_e;
        upperBounds = upperBounds_e;
        speed = speed_e;
        raiseBoundErrors = false;
    }

    public LiftController(int lowerBounds_e, int upperBounds_e) {
        lowerBounds = lowerBounds_e;
        upperBounds = upperBounds_e;
        speed = 100;
        raiseBoundErrors = false;
    }

    public void Init(OpMode opMode, String motor) {
        lift = (DcMotorEx) opMode.hardwareMap.dcMotor.get(motor);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPosition(int height) throws BruhException {
        String message = "nothing";
        if ((lowerBounds <= height) && (height <= upperBounds)) {
            
            lift.setTargetPosition(height);
            lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift.setVelocity(speed);
            
        } else if (height < lowerBounds){
            message = String.format("%s is below the lower bounds of %s ticks", height, lowerBounds);
        } else {
            message = String.format("%s is above the upper bounds of %s ticks", height, upperBounds);
        }
        if(!message.equals("nothing")){
            if(raiseBoundErrors)throw new BruhException(message);
        }
    }

    public void setSpeed(double eSpeed){
        speed = eSpeed;
    }

    public void toLowerBounds(){
        lift.setTargetPosition(lowerBounds);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(speed);
    }

    public void toUpperBounds(){
        lift.setTargetPosition(upperBounds);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(speed);
    }

    private int lowerBounds;
    private int upperBounds;
    private double speed;
    private boolean raiseBoundErrors;

    private DcMotorEx lift;
}