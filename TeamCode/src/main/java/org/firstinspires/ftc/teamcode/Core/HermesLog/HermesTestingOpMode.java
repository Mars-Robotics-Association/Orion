package org.firstinspires.ftc.teamcode.Core.HermesLog;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;

@TeleOp(name="Hermes Testing", group="Testing")
//@Disabled
public class HermesTestingOpMode extends OpMode
{
    HermesLog log;

    @Override
    public void init() {
        log = new HermesLog("HERMES TESTING", 50, this);
    }

    @Override
    public void start() {
        log.start();
    }

    @Override
    public void loop() {
        RobotPose robotPose = new RobotPose(Math.sin(getRuntime()*0.5)*70,
                Math.cos(getRuntime())*70,
                fixAngle(Math.cos(getRuntime())*70),
                (Math.sin(getRuntime()*0.5)*70)+2,
                (Math.cos(getRuntime())*70)+2,
                fixAngle(Math.cos(getRuntime())*70));
        //Base64Image imgToSend = new Base64Image("eggs");
        Object[] data = {robotPose};
        log.addData(data);
        log.update();
    }

    double fixAngle(double angle){
        while(angle>180) {
            angle -= 360;
        }
        while(angle<=-180) {
            angle += 360;
        }
        return angle;
    }
}
