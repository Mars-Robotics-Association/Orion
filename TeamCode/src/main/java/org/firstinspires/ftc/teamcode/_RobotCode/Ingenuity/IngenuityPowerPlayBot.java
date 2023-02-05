package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;

@Config
public class IngenuityPowerPlayBot extends BaseRobot {
    public enum SignalColor {
        BLUE,
        RED,
        GREEN
    }

    ////Dependencies////
    OpMode opMode;
    HermesLog log;

    //Mechanical Components
    IngenuityPayload payload;
    IngenuityNavigation navigator;

    // Gripper
    Servo gripperServo;
    public static double servoTargetClosed = 0.37;//closed
    public static double servoTargetOpen = 0.7;//open
    public static double servoTarget = servoTargetOpen;

    ColorSensor colorSensor;
    DistanceSensor sensorDistance;

    //Misc
    FtcDashboard dashboard;

    private double armStartPos = 0.0;
    private int armSetpointIdx = 0;
    private double[] armStops = {0.0, 0.1555, 0.24177, 0.3576};

    public IngenuityPowerPlayBot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE, new Pose(0, 0, 0), usePayload, useChassis, useNavigator);
        opMode = setOpMode;

        log = new HermesLog("Ingenuity", 50, opMode);
        dashboard = FtcDashboard.getInstance();

        if (USE_CHASSIS) {
            //sensors
            //DistanceSensor portDistance = opMode.hardwareMap.get(DistanceSensor.class, "port distance");
            //DistanceSensor starboardDistance = opMode.hardwareMap.get(DistanceSensor.class, "starboard distance");
            //ColorSensor colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color sensor");

            //initialize the chassis & navigator
            setChassisProfile(new _ChassisProfile());
            navigator = new IngenuityNavigation(opMode, this, null, null, null);
        }

        if (USE_PAYLOAD) {
            //intake
            gripperServo = opMode.hardwareMap.servo.get("gripper");
            colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
            sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "colorSensor");

            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("armMotor");
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            payload = new IngenuityPayload(opMode, armMotor);
        }

        if (USE_NAVIGATOR) {

        }
    }

    //SETUP METHODS//
    public void init() {

    }

    public void start() {
        getChassis().startChassis();
        getNavigator().setMeasuredPose(0, 0, 0);
        log.start();
    }

    public void update() {

        if (USE_NAVIGATOR) {
            navigator.update();
            RobotPose robotPose = new RobotPose(navigator.getTargetPose()[0],
                    navigator.getTargetPose()[1], navigator.getTargetPose()[2],
                    navigator.getMeasuredPose().getX(), navigator.getMeasuredPose().getY(), navigator.getMeasuredPose().getHeading());
            //converts camera footage to base 64 for gui
            //Base64Image cameraData = new Base64Image(
            //camera.convertBitmapToBase64(camera.shrinkBitmap(camera.getImage(),240,135),0));
            Object[] data = {robotPose};
            log.addData(data);
            log.Update();
        }

        if (USE_PAYLOAD) {
            gripperServo.setPosition(servoTarget);

        }
    }

    //make sure to stop everything!
    public void stop() {
        if (USE_CHASSIS) {
            navigator.getChassis().stop();
        }
    }

    public void toggleGripper() {
        if (servoTarget == servoTargetClosed) {
            servoTarget = servoTargetOpen;
        } else servoTarget = servoTargetClosed;
    }

    public void ensureGripperOpen() {
        servoTarget = servoTargetOpen;
        update();
    }

    public void ensureGripperClosed() {
        servoTarget = servoTargetClosed;
        update();
    }

    public SignalColor readSignal() {
        SignalColor result = SignalColor.BLUE;
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
        float hue = hsvValues[0];
        opMode.telemetry.addData("hue ", hue);
        if (hue > 300 || hue < 90) {//red wraps back around: it's centered on 0/360 degrees
            result = SignalColor.RED;
        } else if (hue >= 90 && hue < 190) {
            result = SignalColor.GREEN;
        }
        opMode.telemetry.addData("result", result);
        return result;
    }

    private int lastStopUp() {
        double curPos = getPayload().getArm().getPosition();
        int i;
        for (i = 0; i < armStops.length; i++) {
            if (curPos < armStops[i]) return i;
        }
        return i;
    }

    private int nextStopDown() {
        double curPos = getPayload().getArm().getPosition();
        int i;
        for (i = armStops.length - 1; i >= 0; i--) {
            if (curPos > armStops[i]) return i;
        }
        return i;
    }

    public void raiseArmToNextStop() {
        if (armSetpointIdx == -1) {
            armSetpointIdx = lastStopUp();
        }
        if (this.armSetpointIdx < 3) {
            armSetpointIdx += 1;
            moveArmToStop(armSetpointIdx);
        }
    }

    public void lowerArmToNextStop() {
        if (armSetpointIdx == -1) {
            armSetpointIdx = nextStopDown() + 1;
        }
        if (armSetpointIdx > 0) {
            armSetpointIdx -= 1;
            moveArmToStop(armSetpointIdx);
        }
    }

    public void resetArmHomePosition(double pos) {
        armStartPos = pos;
        getPayload().ResetArmHomePosition(pos);
    }

    public void resetArmStateMachine() {
        armSetpointIdx = -1;
    }

    public void moveArmToStop(int stop) {
        getPayload().getArm().goToPosition(armStops[stop] - armStartPos);
    }

    public void waitForGripperOpen() {
        while (gripperServo.getPosition() < servoTargetOpen - 0.05) {
            update();
        }
    }

    public void waitForGripperClosed() {
        while (gripperServo.getPosition() > servoTargetOpen + 0.05) {
            update();
        }
    }


    public IngenuityNavigation getNavigator() {
        return navigator;
    }

    public MecanumChassis getChassis() {
        return navigator.getChassis();
    }

    public IngenuityPayload getPayload() {
        return payload;
    }
}
