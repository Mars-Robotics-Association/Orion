package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
;
import org.firstinspires.ftc.teamcode.WhiteWolf.WhiteWolfNavigator;

@TeleOp(name = "WHITE WOLF TEST", group = "Testing")
@Disabled
public class WhiteWolfTest extends OpMode {

    WhiteWolfNavigator wwn = new WhiteWolfNavigator(hardwareMap, "TeleOp");

    @Override
    public void init() {
        wwn.init();
    }

    @Override
    public void loop() {
        wwn.loop();
    }

}
