package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


class LiftController {
    public LiftController(int upperBounds_e, double speed_e, boolean raiseBoundErrors_e) {
        upperBounds = upperBounds_e;
        speed = speed_e;
        raiseBoundErrors = raiseBoundErrors_e;
    }

    public LiftController(int upperBounds_e, boolean raiseBoundErrors_e) {
        upperBounds = upperBounds_e;
        speed = 0.1;
        raiseBoundErrors = raiseBoundErrors_e;
    }

    public LiftController(int upperBounds_e, double speed_e) {
        upperBounds = upperBounds_e;
        speed = speed_e;
        raiseBoundErrors = false;
    }

    public LiftController(int upperBounds_e) {
        upperBounds = upperBounds_e;
        speed = 0.1;
        raiseBoundErrors = false;
    }

    public void Init(OpMode opMode, String motor) {
        lift = (DcMotorEx) opMode.hardwareMap.dcMotor.get(motor);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftOrion = (DcMotor) opMode.hardwareMap.dcMotor.get(motor);
        liftOrion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public void go(double myspeed){
        liftOrion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftOrion.setPower(myspeed);

    }

    public void setSpeed(double eSpeed){
        speed = eSpeed;
    }

    public void rawMove(double eSpeed) {
        liftOrion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftOrion.setPower(eSpeed);
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

    public void LockArm(){
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(speed);
    }

    private final int lowerBounds = 0;
    private final int upperBounds;
    private double speed;
    private boolean raiseBoundErrors;

    private DcMotorEx lift;
    private DcMotor liftOrion ;
}