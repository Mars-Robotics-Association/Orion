package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode._RobotCode.Archived.MecChassis;
import org.firstinspires.ftc.teamcode._RobotCode.Archived.MecanumBaseControl;
import org.opencv.core.Mat;

/**
 * Control class for the Belinda Robot. Controls payload.
 * Required to run: Phones | REV Hub | Belinda Chassis
 * Suggested to run: Shooter | Intake | Odometry | Webcam
 */
//The class used to control the demobot. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the demobot.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit

@Config
public class IngenuityControl extends MecanumBaseControl {
    ////Dependencies////
    //Mechanical Components
    private IngenuityLift lift;
    private IngenuityIntakeController intake;
    private IngenuityDuckController duckController;

    ////Variables////
    private double duckSpeed = 80;

    //Calibration
    private double levelPitchThreshold = 5;

    /**
     * @param setOpMode    pass the opmode running this down to access hardware map
     * @param useChassis   whether to use the chassis of the robot
     * @param usePayload   whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public IngenuityControl(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new IngenuityNavProfile(), new HermesLog("Ingenuity", 500, setOpMode), useChassis, usePayload, useNavigator);
    }

    //SETUP METHODS//
    public void Init() {
        super.InitCoreRobotModules();

        super.USE_CHASSIS = false;

        if (USE_PAYLOAD) {
            lift = new IngenuityLift(opMode, opMode.hardwareMap.dcMotor.get("liftMotor"));

            intake = new IngenuityIntakeController(opMode.hardwareMap.servo.get("intakeServo"));

            duckController = new IngenuityDuckController(opMode.hardwareMap.servo.get("duckServo"), duckSpeed);
        }
    }

    public void Start() {
        super.StartCoreRobotModules();
    }

    public boolean IsRobotLevel() {
        double pitch = imu.GetRawAngles().secondAngle;
        opMode.telemetry.addData("Robot pitch: ", pitch);

        return !(Math.abs(pitch) > 5);
    }

    public IngenuityLift GetLift() {
        return lift;
    }

    public IngenuityIntakeController GetIntake() {
        return intake;
    }

    public IngenuityDuckController GetDuck() {
        return duckController;
    }

    //ADVANCED DRIVE CONTROL//
    enum DriveDirection {
        N,
        S,
        E,
        W,
        NE,
        NW,
        SE,
        SW
    }

    int ResolveDriveDirection(DriveDirection direction) {
        int ticks = 0;
        switch (direction) {
            case N:
                ticks = 0;
                break;
            case NE:
                ticks = 1;
                break;
            case E:
                ticks = 2;
                break;
            case SE:
                ticks = 3;
                break;
            case S:
                ticks = 4;
                break;
            case SW:
                ticks = 5;
                break;
            case W:
                ticks = 6;
                break;
            case NW:
                ticks = 7;
                break;
        }

        return ticks * 45;
    }

    public void rawDriveFor(int duration, DriveDirection direction, double speed, double turnOffset) throws InterruptedException {
        RawDrive(ResolveDriveDirection(direction), speed, turnOffset);
        Thread.sleep(duration);
        RawDrive(0, 0, 0);
    }

    public void rawDriveFor(double duration, DriveDirection direction, double speed, double turnOffset) throws InterruptedException {
        RawDrive(ResolveDriveDirection(direction), speed, turnOffset);
        Thread.sleep(ToMillis(duration));
        RawDrive(0, 0, 0);
    }

    public void rawDriveFor(int duration, double inputAngle, double speed, double turnOffset) throws InterruptedException {
        RawDrive(inputAngle, speed, turnOffset);
        Thread.sleep(duration);
        RawDrive(0, 0, 0);
    }

    public void rawDriveFor(double duration, double inputAngle, double speed, double turnOffset) throws InterruptedException {
        RawDrive(inputAngle, speed, turnOffset);
        Thread.sleep(ToMillis(duration));
        RawDrive(0, 0, 0);
    }

    public long ToMillis(double seconds) {
        double millis = seconds * 1000;
        return (long) millis;
    }
}
