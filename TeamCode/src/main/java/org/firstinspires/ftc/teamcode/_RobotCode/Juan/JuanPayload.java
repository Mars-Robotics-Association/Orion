package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import android.graphics.Bitmap;

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

    enum LiftMode {
        VERSION_1(200,1800,3100,4500),
        VERSION_2(0  ,0   ,0,   0   );

        final int[] positions;

        LiftMode(int a,int b,int c,int d){
            positions = new int[]{a, b, c, d};
        }
    }

    enum LiftHeight {
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH
    }

    static class LiftController extends Controller{
        private double power;
        private final DcMotor motor;

        double c = 5;

        int computeHeight(LiftHeight height){
            return Juan.LIFT_MODE.positions[height.ordinal()];
        }

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

        public void goToPreset(LiftHeight height){
            motor.setTargetPosition(computeHeight(height));
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

        void printTelemetry() {
            Telemetry telemetry = getTelemetry();
            telemetry.addData("Gripper State", state == GripperState.OPEN ? "OPEN" : "CLOSED");
            telemetry.addData("Position", servo.getPosition());
            telemetry.addData("Direction", servo.getDirection());
        }
    }

    static class SleeveScanner {
        private final SleeveReader sleeveReader;
        private final Camera camera;
        private final FtcDashboard dashboard;
        private final FunctionStandIn<Bitmap, Bitmap> scanLambda;

        SleeveScanner(Camera camera) {
            this.camera = camera;
            this.sleeveReader = new SleeveReader();
            this.dashboard = FtcDashboard.getInstance();

            FunctionStandIn<Bitmap, Mat> funcA = camera::convertBitmapToMat;
            FunctionStandIn<Mat, Mat>    funcB = sleeveReader::processFrame;
            FunctionStandIn<Mat, Bitmap> funcC = camera::convertMatToBitMap;

            scanLambda = funcA.andThen(funcB).andThen(funcC);
        }

        public Camera getCamera() {
            return camera;
        }

        SleeveColor[] colorArray = SleeveColor.values();

        public SleeveColor runScan() throws InterruptedException {
            dashboard.sendImage(scanLambda.apply(camera.getImage()));

            Arrays.sort(colorArray, SleeveColor.comparator);
            return colorArray[0];
        }
    }

    OpMode opMode;

    //initializer
    public JuanPayload(OpMode opMode, boolean useScanner, DcMotor lift, Servo gripper, double liftPower, Camera camera) {
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
