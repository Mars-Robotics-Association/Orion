package org.firstinspires.ftc.teamcode.Core.HermesLog;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.Base64Image;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;

@TeleOp(name="Hermes Testing", group="Testing")
@Disabled
public class HermesTestingOpMode extends OpMode
{
    HermesLog log;
    RobotPose poseToSend;
    Base64Image imgToSend;
    @Override
    public void init() {
        log = new HermesLog("HERMES TESTING", 500, this);
    }

    @Override
    public void start() {
        log.Start();
    }

    @Override
    public void loop() {
        poseToSend = new RobotPose(Math.random(), Math.random(), Math.random());
        imgToSend = new Base64Image("eggs");
        Object[] data = {poseToSend};
        log.AddData(data);
        log.Update();
    }
}
