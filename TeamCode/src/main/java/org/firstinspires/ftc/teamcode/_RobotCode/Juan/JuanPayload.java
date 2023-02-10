package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import android.graphics.Bitmap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCV;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

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

    enum LiftMode {
        VERSION_1(new int[]{   0,1800,3100,4500, 800, 1500, 2100, 3500, 650, 650}),
        VERSION_2(new int[]{   0,0,   0,   0, 0, 0, 0, 0, 0, 0});

        final int[] positions;

        LiftMode(int[] values){
            positions = values;
        }
    }

    enum LiftHeight {
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH,
        CRUISE,
        LOW_RELEASE,
        MEDIUM_RELEASE,
        HIGH_RELEASE,
        STACK_1,
        STACK_2,
    }

    static class LiftController extends Controller{
        private final DcMotor motor;

        public boolean isDoneMoving(){
            return Math.abs( motor.getCurrentPosition() - motor.getCurrentPosition() ) > 50;
        }

        int computeHeight(LiftHeight height){
            return Juan.LIFT_MODE.positions[height.ordinal()];
        }

        public LiftController(JuanPayload payload, DcMotor motor) {
            super(payload);
            this.motor = motor;
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

        public void goToPreset(LiftHeight height){
            motor.setTargetPosition(computeHeight(height));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
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
        private final ColorSensor gripperColor;

        GripperController(JuanPayload payload, Servo servo, ColorSensor legoColor, ColorSensor gripperColor){
            super(payload);
            this.servo = servo;
            this.gripperColor = gripperColor;

            isBlue = legoColor.blue() > legoColor.red();
        }

        private boolean isBlue(ColorSensor sensor){
            return sensor.blue() > sensor.red();
        }

        final boolean isBlue;

        private final Servo servo;

        public Servo getServo(){return servo;}

        private GripperState state = GripperState.CLOSED;
        private final double openPos = 0.55;
        private final double closePos = 0.7;

        private final double increment = 0.005;

        public void grab(){
            if(isBlue != isBlue(gripperColor))return;
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

        void printTelemetry() {
            Telemetry telemetry = getTelemetry();
            telemetry.addData("Gripper State", state == GripperState.OPEN ? "OPEN" : "CLOSED");
            telemetry.addData("Position", servo.getPosition());
            telemetry.addData("Direction", servo.getDirection());
        }
    }

    static class SleeveScanner extends Controller {
        private final Camera camera;

        SleeveScanner(JuanPayload payload, Camera camera) {
            super(payload);
            this.camera = camera;
        }

        public SleeveColor scan() throws InterruptedException {
            Camera c = camera;

            Bitmap input = camera.getImage();
            Mat mat = OpenCV.convertBitmapToMat(input);

            float[] bounds = Juan.SCAN_BOUNDS;

            Mat in = new Mat(mat, new Rect(
                    (int) (mat.width() * bounds[0]),
                    (int) (mat.height()* bounds[1]),
                    (int) (mat.width() * bounds[2]),
                    (int) (mat.height()* bounds[3])
            ));

            Bitmap bitmap = c.shrinkBitmap(OpenCV.convertMatToBitMap(in), 50, 50);
            in = c.convertBitmapToMat(bitmap);

            FtcDashboard.getInstance().sendImage(bitmap);

            int[] results = {0, 0, 0};

            for(SleeveColor color : SleeveColor.values()){
                Mat isolate = c.isolateColor(in, color.highColor, color.lowColor);
                results[color.ordinal()] = c.countPixels(c.convertMatToBitMap(isolate));
            }

            int greenCount  = results[0];
            int orangeCount = results[1];
            int purpleCount = results[2];

            Telemetry telemetry = getTelemetry();

            telemetry.addData("Green Pixels", greenCount);
            telemetry.addData("Orange Pixel", orangeCount);
            telemetry.addData("Purple Pixel", purpleCount);

            if(greenCount>purpleCount&&greenCount>orangeCount){
                return SleeveColor.GREEN;
            } else if(orangeCount>greenCount&&orangeCount>purpleCount){
                return SleeveColor.ORANGE;
            } else{
                return SleeveColor.PURPLE;
            }
        }

        @Override
        void printTelemetry() {

        }
    }

    OpMode opMode;

    //initializer
    public JuanPayload(OpMode opMode, HardwareMap h) {
        this.opMode = opMode;

        liftController = new LiftController(this, h.dcMotor.get("lift"));
        gripperController = new GripperController(this,
                h.servo.get("gripper"),
                h.colorSensor.get("lego"),
                null//h.colorSensor.get("gripperColor")
        );
        sleeveScanner = new SleeveScanner(this,
                new Camera(opMode, "Webcam 1")
        );
    }

    private final LiftController liftController;
    private final GripperController gripperController;
    private final SleeveScanner sleeveScanner;
    public LiftController getLift(){return liftController;}
    public GripperController getGripper(){return gripperController;}
    public SleeveScanner getScanner(){return sleeveScanner;}

    //print telemetry
    public void printTelemetry(){
        opMode.telemetry.addLine("<LIFT>");
        liftController.printTelemetry();
        opMode.telemetry.addLine("<GRIPPER>");
        gripperController.printTelemetry();
        opMode.telemetry.addLine("<SLEEVE>");
        sleeveScanner.printTelemetry();
    }
}
