package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.opencv.core.Mat;

import java.util.Arrays;

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
        BOTTOM(200),
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

        double c = 5;

        LiftController(JuanPayload payload, DcMotor motor, double power) {
            super(payload);
            this.motor = motor;
            this.power = power;
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        public void manualMove(int direction){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(direction);
        }
    }

    enum GripperState{
        OPEN,
        CLOSED
    }

    static class GripperController extends Controller{
        GripperController(JuanPayload payload, Servo servo){
            super(payload);
            this.servo = servo;
        }

        private final Servo servo;

        public Servo getServo(){return servo;}

        private GripperState state = GripperState.CLOSED;
        private final double openPos = 0.55;
        private final double closePos = 0.7;

        private final double increment = 0.005;

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

    static class SleeveScanner{
        private final SleeveReader sleeveReader;
        private final Camera camera;
        private final FtcDashboard dashboard;

        SleeveScanner(Camera camera){
            this.camera = camera;
            this.sleeveReader = new SleeveReader();
            this.dashboard = FtcDashboard.getInstance();
        }

        public Camera getCamera(){
            return camera;
        }

        static class ColorCertaintyTuple implements Comparable {
            public SleeveColor color;
            public double certainty;

            ColorCertaintyTuple(SleeveColor color, double certainty){
                this.color = color;
                this.certainty = certainty;
            }

            @Override
            public int compareTo(Object o) {
                ColorCertaintyTuple tuple = (ColorCertaintyTuple) o;
                return Double.compare(this.certainty, tuple.certainty);
            }
        }

        private ColorCertaintyTuple[] colorTuples = {
                new ColorCertaintyTuple(SleeveColor.GREEN, 0),
                new ColorCertaintyTuple(SleeveColor.ORANGE, 0),
                new ColorCertaintyTuple(SleeveColor.PURPLE, 0)
        };

        public ColorCertaintyTuple runScan() throws InterruptedException {
            Mat mat = new Mat();
            Mat combined = new Mat();
            camera.convertBitmapToMat(camera.GetImage());

            Mat mat1 = testColor(mat, SleeveColor.GREEN);
            colorTuples[0].color = SleeveColor.GREEN;
            colorTuples[0].certainty = sleeveReader.certainty;
            Mat mat2 = testColor(mat, SleeveColor.ORANGE);
            colorTuples[1].color = SleeveColor.ORANGE;
            colorTuples[1].certainty = sleeveReader.certainty;
            Mat mat3 = testColor(mat, SleeveColor.PURPLE);
            colorTuples[2].color = SleeveColor.PURPLE;
            colorTuples[2].certainty = sleeveReader.certainty;

            sleeveReader.joinComparisons(mat, mat1, mat2, mat3, combined);
            dashboard.sendImage(camera.convertMatToBitMap(combined));

            return colorTuples[0];
        }

        private Mat testColor(Mat input, SleeveColor color){
            sleeveReader.setRange(
                    color.highColor,
                    color.lowColor
            );

            return sleeveReader.processFrame(input);
        }

        enum ScanResult{
            GREEN,
            ORANGE,
            PURPLE,
            INVALID;

            boolean isInvalid(){
                return this == ScanResult.INVALID;
            }

            SleeveColor unwrap(){
               switch (this) {
                   case GREEN:
                       return SleeveColor.GREEN;
                   case ORANGE:
                       return SleeveColor.ORANGE;
                   case PURPLE:
                       return SleeveColor.PURPLE;
               }

               throw new RuntimeException("No valid color found.");
            }
        }

        ScanResult lastColor = ScanResult.INVALID;
    }

    OpMode opMode;

    //initializer
    public JuanPayload(OpMode opMode, DcMotor lift, Servo gripper, double liftPower, Camera camera) {
        this.opMode = opMode;

        liftController = new LiftController(this, lift, liftPower);
        gripperController = new GripperController(this, gripper);
        sleeveScanner = new SleeveScanner(camera);
    }

    private final LiftController liftController;
    private final GripperController gripperController;
    private final SleeveScanner sleeveScanner;
    public LiftController getLift(){return liftController;}
    public GripperController getGripper(){return gripperController;}
    public SleeveScanner getScanner(){return sleeveScanner;}

    //print telemetry
    public void printTelemetry(){
        opMode.telemetry.addLine("----PAYLOAD----");
        opMode.telemetry.addLine("<LIFT>");
        liftController.printTelemetry();
        opMode.telemetry.addLine("<GRIPPER>");
        gripperController.printTelemetry();
    }
}
