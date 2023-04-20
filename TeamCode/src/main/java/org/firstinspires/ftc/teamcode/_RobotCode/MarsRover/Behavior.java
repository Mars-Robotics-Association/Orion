package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

public class Behavior {
    private static final ArrayList<Behavior> BEHAVIOR_ARRAY = new ArrayList<>();

    public static void systemInit(OpMode opmode, Behavior... behaviors){
        BEHAVIOR_ARRAY.clear();

        BEHAVIOR_ARRAY.addAll(Arrays.asList(behaviors));

        try {
            for (Behavior behavior : behaviors) {
                behavior.setupFields(opmode);

                behavior.init();
            }
        }catch(Exception error){
            RobotLog.e(error.toString());

        }
    }

    public static void systemStart(){
        for(Behavior behavior : BEHAVIOR_ARRAY){
            behavior.start();
        }
    }

    public static void systemUpdate(){
        for(Behavior behavior : BEHAVIOR_ARRAY){
            behavior.update();
        }
    }

    public static void systemStop(){
        for(Behavior behavior : BEHAVIOR_ARRAY){
            behavior.stop();
        }
    }

    protected OpMode opMode;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;

    private void setupFields(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
    }

    protected void init() throws Exception{}


    protected void start(){}

    protected void update(){}

    protected void stop(){}

    protected <T extends Behavior> T getBehavior(Class<T> tClass){
        for (Behavior behavior : BEHAVIOR_ARRAY){
            if(tClass.isInstance(behavior))return tClass.cast(behavior);
        }
        return null;
    }
}
