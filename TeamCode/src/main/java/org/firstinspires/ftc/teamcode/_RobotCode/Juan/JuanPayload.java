package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;

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

    static class LiftController extends Controller{
        private DcMotor motor;

        enum PresetHeight{
            BOTTOM(10),
            LOW(173),
            MEDIUM(1566),
            HIGH(2349);

            final int position;

            PresetHeight(int position){
                this.position = position;
            }
        }

        LiftController(JuanPayload payload, DcMotor motor) {
            super(payload);
            this.motor = motor;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        @Override
        void printTelemetry() {
            Telemetry telemetry = getTelemetry();
            telemetry.addData("Current power: ", motor.getPower());
            telemetry.addData("Current position: ", motor.getCurrentPosition());
            telemetry.addData("Target position: ", motor.getTargetPosition());
        }

        public void setPower(double power){
            motor.setPower(power);
        }

        public void goToPreset(PresetHeight height){
            motor.setTargetPosition(height.position);
        }
    }

    static class GripperController extends Controller{
        enum GripperState{
            OPEN,
            CLOSED
        }

        GripperController(JuanPayload payload, Servo servo) {
            super(payload);
            this.servo = servo;
        }

        private final Servo servo;

        private GripperState state = GripperState.OPEN;
        private final double openPos = .2;
        private final double closePos = .1;

        public void grab(){
            servo.setPosition(closePos);
            state = GripperState.OPEN;
        }

        public void release(){
            servo.setPosition(openPos);
            state = GripperState.CLOSED;
        }

        public void toggle(){
            switch(state){
                case OPEN:
                    grab();
                    break;
                case CLOSED:
                    release();
            }
        }

        @Override
        void printTelemetry() {
            Telemetry telemetry = getTelemetry();
            telemetry.addData("Gripper State", state == GripperState.OPEN ? "OPEN" : "CLOSED");
            telemetry.addData("Position", servo.getPosition());
            telemetry.addData("Direction", servo.getDirection());
        }
    }

    OpMode opMode;

    //initializer
    public JuanPayload(OpMode opMode, DcMotor lift, Servo gripper) {
        this.opMode = opMode;

        liftController = new LiftController(this, lift);
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
