package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        enum LiftHeight {
            LOW,
            MEDIUM,
            HIGH
        }

        private EncoderActuator actuator;

        private final double lowPos = 173;
        private final double mediumPos = 1566;
        private final double highPos = 2349;

        LiftController(JuanPayload payload, DcMotor motor) {
            super(payload);
            _LiftProfile profile = new _LiftProfile(motor);
            actuator = new EncoderActuator(payload.opMode, profile);
        }

        @Override
        void printTelemetry() {
            Telemetry telemetry = getTelemetry();
            telemetry.addData("Current final position: ", actuator.getFinalPosition());
        }

        public void goToAbsoluteTop(){actuator.goToMax();lockActuator();}
        public void goToAbsoluteBottom(){actuator.goToMin();}

        public void lockActuator(){actuator.lock();}

        public void goToPreset(LiftHeight preset){
            double height = 0;

            switch(preset) {
                case LOW:
                    height = lowPos;
                    break;
                case MEDIUM:
                    height = mediumPos;
                    break;
                case HIGH:
                    height = highPos;
                    break;
            }

            actuator.goToPosition(height);
            lockActuator();
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
            servo.setPosition(0);
        }

        private final Servo servo;

        private GripperState state = GripperState.OPEN;
        private double openPos = .2;
        private double closePos = -0.3;

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
        opMode.telemetry.addLine("!!LIFT");
        liftController.printTelemetry();
        opMode.telemetry.addLine("!!GRIPPER");
        gripperController.printTelemetry();
    }
}
