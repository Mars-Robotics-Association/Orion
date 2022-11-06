package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@Config
class JuanNavigation
{
    ////DEPENDENCIES////
    private final OpMode opMode;
    private final MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}

    double lastTimeAboveStopThreshold = 0;

    public JuanNavigation(OpMode setOpMode, BaseRobot baseRobot){
        opMode = setOpMode;
        chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("JUAN", 200, setOpMode), baseRobot);
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

    private double driveAngle = 0;
    private double driveMag = 0;
    private double turnAngle = 0;

    private void blendGamepadInputs(
            double driveAngle1,
            double driveAngle2,
            double driveMag1,
            double driveMag2,
            double turnAngle1,
            double turnAngle2
    ){
        boolean useP1ForDrive = Math.abs(driveMag1) > Math.abs(driveMag2);
        boolean useP1ForTurn = Math.abs(turnAngle1) > Math.abs(turnAngle2);

        driveAngle = useP1ForDrive ? driveAngle1 : driveAngle2;
        driveMag = useP1ForDrive ? driveMag1 : driveMag2;
        turnAngle = useP1ForTurn ? turnAngle1 : turnAngle2;
    }

    public void dualGamepadDrive(ControllerInput input1, ControllerInput input2, double driveSpeed, double turnSpeed){
        blendGamepadInputs(
                input1.CalculateLJSAngle(),
                input2.CalculateLJSAngle(),
                input1.CalculateLJSMag(),
                input2.CalculateLJSMag(),
                input1.GetRJSX(),
                input2.GetRJSX()
        );

        //MOVE if left joystick magnitude > 0.1
        if (driveMag > 0.1) {
            chassis.rawDrive(input1.CalculateLJSAngle(), input1.CalculateLJSMag() * driveSpeed, input1.GetRJSX() * turnSpeed);//drives at (angle, speed, turnOffset)
            opMode.telemetry.addData("Moving at ", input1.CalculateLJSAngle());
        }
        //TURN if right joystick magnitude > 0.1 and not moving
        else if (Math.abs(turnAngle) > 0.1) {
            chassis.rawTurn(turnAngle * turnSpeed);//turns at speed according to rjs1
            opMode.telemetry.addData("Turning", true);
        }
        else {
            chassis.setMotorSpeeds(0,0,0,0);
        }
    }
}
