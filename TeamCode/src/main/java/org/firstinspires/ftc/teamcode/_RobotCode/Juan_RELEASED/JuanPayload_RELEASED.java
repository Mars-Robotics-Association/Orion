package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.opencv.core.Mat;

class JuanPayload_RELEASED
{
    static abstract class Controller{
        private JuanPayload_RELEASED payload;

        protected Telemetry getTelemetry(){return payload.opMode.telemetry;}
        protected OpMode getOpMode(){return payload.opMode;}

        Controller(JuanPayload_RELEASED payload){
            this.payload = payload;
        }

        abstract void printTelemetry();
    }

    enum LiftMode {
        VERSION_1(   0,1800,3100,4500),
        VERSION_2(   0,   0,   0,   0);

        final int[] positions;

        LiftMode(int a,int b,int c, int d) { positions = new int[]{a,b,c,d}; }
    }

    enum LiftHeight {
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH
    }

   /* enum PresetHeight{
        BOTTOM(200),
        LOW(1800),
        MEDIUM(3100),
        HIGH(4500);

        final int position;

        PresetHeight(int position){
            this.position = position;
        }
    }
    */

    static class LiftController extends Controller{
        private double power;
        private final DcMotor motor;

        double c = 5;

        int computeHeight(LiftHeight height) {
            // Return encoder position from LiftMode for enumerated value in LiftHeight (height)
            return Juan_RELEASED.LIFT_MODE.positions[height.ordinal()];
        }

        LiftController(JuanPayload_RELEASED payload, DcMotor motor, double power) {
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
        GripperController(JuanPayload_RELEASED payload, Servo servo){
            super(payload);
            this.servo = servo;
        }

        private final Servo servo;

        public Servo getServo(){return servo;}

        private GripperState state = GripperState.CLOSED;
        private final double openPos = 0.53;
        private final double closePos = 0.675;

        // Specify the increment for minor manual adjustment of gripper by Drive Team
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
        private final SleeveReader_RELEASED sleeveReaderRELEASED;
        private final Camera camera;
        private final FtcDashboard dashboard;

        SleeveScanner(Camera camera){
            this.camera = camera;
            this.sleeveReaderRELEASED = new SleeveReader_RELEASED();
            this.dashboard = FtcDashboard.getInstance();
        }

        public Camera getCamera(){
            return camera;
        }

        static class ColorCertaintyTuple implements Comparable {
            public SleeveColor_RELEASED color;
            public double certainty;

            ColorCertaintyTuple(SleeveColor_RELEASED color, double certainty){
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
                new ColorCertaintyTuple(SleeveColor_RELEASED.GREEN, 0),
                new ColorCertaintyTuple(SleeveColor_RELEASED.ORANGE, 0),
                new ColorCertaintyTuple(SleeveColor_RELEASED.PURPLE, 0)
        };

        public ColorCertaintyTuple runScan() throws InterruptedException {
            Mat mat = new Mat();
            Mat combined = new Mat();
            camera.convertBitmapToMat(camera.getImage());

            Mat mat1 = testColor(mat, SleeveColor_RELEASED.GREEN);
            colorTuples[0].color = SleeveColor_RELEASED.GREEN;
            colorTuples[0].certainty = sleeveReaderRELEASED.certainty;
            Mat mat2 = testColor(mat, SleeveColor_RELEASED.ORANGE);
            colorTuples[1].color = SleeveColor_RELEASED.ORANGE;
            colorTuples[1].certainty = sleeveReaderRELEASED.certainty;
            Mat mat3 = testColor(mat, SleeveColor_RELEASED.PURPLE);
            colorTuples[2].color = SleeveColor_RELEASED.PURPLE;
            colorTuples[2].certainty = sleeveReaderRELEASED.certainty;

            sleeveReaderRELEASED.joinComparisons(mat, mat1, mat2, mat3, combined);
            dashboard.sendImage(camera.convertMatToBitMap(combined));

            return colorTuples[0];
        }

        private Mat testColor(Mat input, SleeveColor_RELEASED color){
            sleeveReaderRELEASED.setRange(
                    color.highColor,
                    color.lowColor
            );

            return sleeveReaderRELEASED.processFrame(input);
        }

        enum ScanResult{
            GREEN,
            ORANGE,
            PURPLE,
            INVALID;

            boolean isInvalid(){
                return this == ScanResult.INVALID;
            }

            SleeveColor_RELEASED unwrap(){
               switch (this) {
                   case GREEN:
                       return SleeveColor_RELEASED.GREEN;
                   case ORANGE:
                       return SleeveColor_RELEASED.ORANGE;
                   case PURPLE:
                       return SleeveColor_RELEASED.PURPLE;
               }

               throw new RuntimeException("No valid color found.");
            }
        }

        ScanResult lastColor = ScanResult.INVALID;
    }

    OpMode opMode;

    //initializer
    public JuanPayload_RELEASED(OpMode opMode, DcMotor lift, Servo gripper, double liftPower, Camera camera) {
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
