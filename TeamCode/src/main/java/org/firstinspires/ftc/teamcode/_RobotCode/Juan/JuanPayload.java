package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class JuanPayload
{
    static abstract class Controller{
        private JuanPayload payload;

        protected Telemetry getTelemetry(){return payload.opMode.telemetry;}
        protected OpMode getOpMode(){return payload.opMode;}

        Controller(JuanPayload payload){
            this.payload = payload;
        }

        abstract void printTelemetry();
    }

    enum PresetHeight{
        BOTTOM(10),
        LOW(1800),
        MEDIUM(3100),
        HIGH(4500);

        final int position;

        PresetHeight(int position){
            this.position = position;
        }
    }

    static class LiftController extends Controller{
        private double power;
        private final DcMotor motor;

        LiftController(JuanPayload payload, DcMotor motor, double power) {
            super(payload);
            this.motor = motor;
            this.power = power;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        void printTelemetry() {
            Telemetry telemetry = getTelemetry();
            telemetry.addData("Current power: ", motor.getPower());
            telemetry.addData("Current position: ", motor.getCurrentPosition());
            telemetry.addData("Target position: ", motor.getTargetPosition());
        }

        public void setSpeed(double speed){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(speed);
        }

        public void reset(){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void goToPreset(PresetHeight height){
            motor.setTargetPosition(height.position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        public void manualMove(){

        }
    }

    enum GripperState{
        OPEN,
        CLOSED
    }

    static class GripperController extends Controller{
        GripperController(JuanPayload payload, Servo servo) {
            super(payload);
            this.servo = servo;
        }

        private final Servo servo;

        public Servo getServo(){return servo;}

        private GripperState state = GripperState.CLOSED;
        private final double openPos = 0;
        private final double closePos = 0.8;

        private final double increment = 0.1;

        public void grab(){
            servo.setPosition(closePos);
            state = GripperState.OPEN;
        }

        public void grabMore(){
            servo.setPosition(servo.getPosition() + increment);
        }

        public void release(){
            servo.setPosition(openPos);
            state = GripperState.CLOSED;
        }

        public void releaseMore(){
            servo.setPosition(servo.getPosition() - increment);
        }

        public double getPosition(){
            return servo.getPosition();
        }

//        public void toggle(){
//            switch(state){
//                case OPEN:
//                    grab();
//                    break;
//                case CLOSED:
//                    release();
//
//            }
//        }

        void printTelemetry() {
            Telemetry telemetry = getTelemetry();
            telemetry.addData("Gripper State", state == GripperState.OPEN ? "OPEN" : "CLOSED");
            telemetry.addData("Position", servo.getPosition());
            telemetry.addData("Direction", servo.getDirection());
        }
    }

    OpMode opMode;

    //initializer
    public JuanPayload(OpMode opMode, DcMotor lift, Servo gripper, double liftPower) {
        this.opMode = opMode;

        liftController = new LiftController(this, lift, liftPower);
        gripperController = new GripperController(this, gripper);
    }

    private final LiftController liftController;
    private final GripperController gripperController;
    public LiftController getLift(){return liftController;}
    public GripperController getGripper(){return gripperController;}

    //print telemetry
    public void printTelemetry(){
        opMode.telemetry.addLine("----PAYLOAD----");
        opMode.telemetry.addLine("<LIFT>");
        liftController.printTelemetry();
        opMode.telemetry.addLine("<GRIPPER>");
        gripperController.printTelemetry();
    }
}
