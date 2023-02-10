package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Ingenuity Autonomous Left", group = "Ingenuity")
@Config
public class IngenuityAutonomousLeft extends IngenuityAutonomous {

    @Override
    protected void posMedJunction() {
        goToPose(27, 0, 45, 2500, true);
    }

    @Override
    protected void posPreStack() {
        goToPose(46.5, 0, -87, 1900, false);
    }

    @Override
    protected void posStack() {
        goToPose(46.5, -16.5, -87, 2500, true);
    }

    @Override
    protected void posPostStack() {
        goToPose(46.5, -2, -87, 1800, false);
    }

    @Override
    protected void posHighJunction() {
        goToPose(53, 7, 36, 2750, true);
    }

    @Override
    protected void posPostHighJunction() {
        goToPose(46, 0, 0, 1200, false);
    }

    @Override
    protected void posLowJunction() {
        goToPose(47, 2.5, -127, 3000, true);
    }
}
