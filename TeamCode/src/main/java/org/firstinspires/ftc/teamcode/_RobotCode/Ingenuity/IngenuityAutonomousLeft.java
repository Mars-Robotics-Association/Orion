package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Ingenuity Autonomous Left", group = "Ingenuity")
@Config
public class IngenuityAutonomousLeft extends IngenuityAutonomous {
    @Override
    protected double mapX(double x) {
        return x;
    }

    @Override
    protected double mapY(double y) {
        return -y;
    }

    @Override
    protected double mapAngle(double angle) {
        return -angle;
    }
}
