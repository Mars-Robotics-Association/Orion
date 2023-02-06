package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Ingenuity Autonomous Right", group = "Ingenuity")
@Config
public class IngenuityAutonomousRight extends IngenuityAutonomous {
    @Override
    protected void posMedJunction() {
        goToPose( 26.5, -1.5, -45, 10000, true);
    }

    @Override
    protected void posPreStack() {
        goToPose(50, 0, 87, 2500, false);
    }

    @Override
    protected void posStack() {
        goToPose(48, -3.5, 127, 10000, true);
    }

    @Override
    protected void posPostStack() {
        goToPose(50, 2, 87, 1200, false);
    }

    @Override
    protected void posHighJunction() {
        goToPose(54, -10.5, -36, 10000, true);
    }

    @Override
    protected void posPostHighJunction() {
        goToPose(49, -5, -36, 1000, false);
    }

    @Override
    protected void posLowJunction() {
        goToPose(48, -3.5, 127, 10000, true);
    }
}
