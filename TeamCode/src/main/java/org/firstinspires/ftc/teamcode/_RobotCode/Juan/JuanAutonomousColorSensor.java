package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@Autonomous(name = "JUAN COLOR SENSOR", group = "JUAN")
@Config
public class JuanAutonomousColorSensor extends LinearOpMode
{
    Juan robot;

    @Override
    public void runOpMode() {
        robot = new Juan(this,true,true, false);
        robot.init();

        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        MecanumChassis chassis = robot.getChassis();
        sleep(500);

        chassis.rawDrive(0, -0.5, 0);

        while ((distanceSensor.getDistance(DistanceUnit.INCH) > 1)) {
            sleep(100);
        }

        chassis.rawDrive(0, 0, 0);

        int     r = colorSensor.red(),
                g = colorSensor.green(),
                b = colorSensor.blue();

        Color color = Color.scan(r, g, b);

        doSomething(color.ordinal() + 1);

    }

    enum Color{
        GREEN (105,217,181,62 ,69,0  ),
        ORANGE(21 ,184,219,0  ,82,154),
        PURPLE(149,165,200,110,88,143),
        INVALID(0 ,0  ,0  ,0,  0 ,0  );

        final float[] components;

        Color(int a, int b, int c, int d, int e, int f){
            components = new float[]{a, b, c, d, e, f};
        }

        static Color scan(int r, int g, int b){
            float[] hsv = {0, 0, 0};
            android.graphics.Color.RGBToHSV(r, g, b, hsv);

            for (Color color : Color.values()){
                if(color.test(hsv))return color;
            }

            return INVALID;
        }

        boolean test(float[] test){
            float[] c = components;

            boolean passH = c[0] < test[0] && test[0] < c[3];
            boolean passS = c[1] < test[1] && test[1] < c[4];
            boolean passV = c[2] < test[2] && test[2] < c[5];

            return passH && passS && passV;
        }
    }

    private void doSomething(int count){
        MecanumChassis chassis = robot.getChassis();
        for(int i=0;i<count;i++){
            chassis.rawTurn(1);
            sleep(500);
        }
    }
}
