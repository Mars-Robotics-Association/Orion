package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.inspection.GamepadInspection;

import java.util.GregorianCalendar;

@Config
class JuanNavigation
{
    ////DEPENDENCIES////
    private final OpMode opMode;
    private final MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}
    public IMU imu;

    public JuanNavigation(OpMode setOpMode, BaseRobot baseRobot){
        opMode = setOpMode;
        chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("JUAN", 200, setOpMode), baseRobot);
        imu = new IMU(opMode);
    }

    public void update(){

    }

    ////UTILITY////
    private double getDistance(double x1, double y1, double x2, double y2){
        double xError = x1-x2;
        double yError = y1-y2;
        return Math.sqrt((xError*xError)+(yError*yError)); //return distance
    }

    private double getMedian(double a, double b){
        return (a + b) / 2;
    }

    final double radToDegrees = 180 / Math.PI;
    final double DegreesToRad = 180 / Math.PI;

    public boolean HEADLESS_MODE = false;

    public boolean toggleHeadless(){
        if(!HEADLESS_MODE)imu.ResetGyro();
        return HEADLESS_MODE = !HEADLESS_MODE;
    }

    public void gamepadDrive(ControllerInput input, double driveSpeed, double turnSpeed) {
        double magnitude = input.CalculateLJSMag();
        double angle = input.CalculateLJSAngle();
        double turn = input.GetRJSX();

        if(HEADLESS_MODE)turn += 0;

        new IMU(opMode);

        //MOVE if left joystick magnitude > 0.1
        if (magnitude > 0.1) {
            chassis.rawDrive(angle, magnitude * driveSpeed, turn * turnSpeed);//drives at (angle, speed, turnOffset)
            opMode.telemetry.addData("Moving at ", input.CalculateLJSAngle());
        }
        //TURN if right joystick magnitude > 0.1 and not moving
        else if (Math.abs(input.GetRJSX()) > 0.1) {
            chassis.rawTurn(turn * turnSpeed);//turns at speed according to rjs1
            opMode.telemetry.addData("Turning", true);
        } else {
            chassis.setMotorSpeeds(0, 0, 0, 0);
        }
    }
}
