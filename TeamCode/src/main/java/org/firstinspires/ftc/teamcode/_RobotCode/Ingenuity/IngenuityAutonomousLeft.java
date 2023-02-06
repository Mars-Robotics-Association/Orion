package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Ingenuity Autonomous Left", group = "Ingenuity")
@Config
public class IngenuityAutonomousLeft extends IngenuityAutonomous {

    @Override
    protected void posMedJunction() {
        goToPose( 26.5, 1.5, 45, 10000, true);
    }

    @Override
    protected void posPreStack() {
        goToPose(47, 0, -87, 2500, false);
    }

    @Override
    protected void posStack() {
        goToPose(47, -14, -87, 10000, true);
    }

    @Override
    protected void posPostStack() {
        goToPose(47, -2, -87, 1200, false);
    }

    @Override
    protected void posHighJunction() {
        goToPose(54, 8, 36, 10000, true);
    }

    @Override
    protected void posPostHighJunction() {
        goToPose(49, 3, 36, 1000, false);
    }

    @Override
    protected void posLowJunction() {
        goToPose(48, 3.5, -127, 10000, true);
    }
}
