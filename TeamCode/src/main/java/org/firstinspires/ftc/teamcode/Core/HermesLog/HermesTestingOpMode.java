package org.firstinspires.ftc.teamcode.Core.HermesLog;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Hermes Testing", group="Testing")
public class HermesTestingOpMode extends OpMode
{
    HermesLog log;
    RobotPose poseToSend;
    imageTesting imgToSend;
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
        imgToSend = new imageTesting("eggs");
        Object[] data = {poseToSend,imgToSend};
        log.AddData(data);
        log.Update();
    }
}
