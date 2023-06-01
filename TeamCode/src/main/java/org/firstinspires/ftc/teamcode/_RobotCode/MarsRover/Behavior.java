package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * The Behavior system itself.
 */
public abstract class Behavior {

    /**
     * Thrown by {@link #getBehavior} when a hardware device could not be found.
     */
    static class BehaviorNotFound extends Exception{
        BehaviorNotFound(String name){
            super(String.format("Behavior %s could not be found", name));
        }
    }

    private static final ArrayList<Behavior> BEHAVIOR_ARRAY = new ArrayList<>();

    private static boolean running = false;
    private static Exception currentException = null;
    private static String crashedInPhase = "???";

    private static boolean transmitException() {
        if (!running) {
            telemetry.clearAll();

            Exception exception = currentException;
            telemetry.addLine("Orbitron crashed at " + crashedInPhase + " Phase");
            telemetry.addData("Cause", exception.getMessage());
            telemetry.addLine("Stack Trace");
            for (StackTraceElement element: exception.getStackTrace()) {
                telemetry.addLine(element.toString());
            }

            telemetry.update();
        }

        return !running;
    }

    public static void sendException(Exception exception, String phase) {
        currentException = exception;
        crashedInPhase = phase;
        running = false;

        RobotLog.e("Orbitron crashed at " + crashedInPhase + " Phase");
        RobotLog.e("Cause", exception.getMessage());
        RobotLog.e("Stack Trace");
        for (StackTraceElement element: exception.getStackTrace()) {
            RobotLog.e(element.toString());
        }

        transmitException();
    }

    public static void sendException(Exception exception) {
        sendException(exception, "Unnamed");
    }

    /**
     * Initializes all the specified Behaviors and calls the {@link #init()} on each behavior.
     * Call this in your OpMode's {@link OpMode#init()}.
     */
    public static void systemInit(OpMode opmode, Behavior... behaviors) {
        BEHAVIOR_ARRAY.clear();

        BEHAVIOR_ARRAY.addAll(Arrays.asList(behaviors));

        opMode = opmode;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        currentException = null;
        running = true;

        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.init();
            }
        }catch(Exception e){
            sendException(e, "Init");
        }

        transmitException();
        telemetry.update();
    }

    /**
     * Calls the {@link #start()} on each behavior.
     * Call this in your OpMode's {@link OpMode#start()}.
     */
    public static void systemStart() {
        if(transmitException())return;

        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.start();
            }
        }catch(Exception e){
            sendException(e, "Start");
        }

        telemetry.update();
    }

    /**
     * Calls the {@link #update()} on each behavior.
     * Call this in your OpMode's {@link OpMode#loop()},
     * or in a while loop inside {@link LinearOpMode#runOpMode()}.
     */
    public static void systemUpdate() {
        if(transmitException())return;

        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.update();
            }
        }catch(Exception e){
            sendException(e, "Update");
        }

        telemetry.update();
    }

    /**
     * Calls the {@link #stop()} on each behavior.
     * Call this in your OpMode's {@link OpMode#stop()}.
     */
    public static void systemStop() {
        if(transmitException())return;

        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.stop();
            }
        }catch(Exception e){
            sendException(e, "Stop");
        }

        telemetry.update();
    }

    public static OpMode opMode;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    /**
     * Called on {@link OpMode#init()}
     */
    protected abstract void init() throws Exception;

    /**
     * Called on {@link OpMode#start()}
     */
    protected abstract void start() throws Exception;

    /**
     * Called on {@link OpMode#loop()} or synthetically by {@linkplain LinearOpMode#runOpMode()}
     */
    protected abstract void update() throws Exception;

    /**
     * Called on {@link OpMode#stop()}
     */
    protected abstract void stop() throws Exception;

    /**
     *
     * @param tClass Hardware class as a variable.
     * @return T if found
     * @param <T> Expected Hardware Type. Should be computed by Java typing system.
     */
    protected static <T extends Behavior> T getBehavior(Class<T> tClass) throws BehaviorNotFound {
        for (Behavior behavior : BEHAVIOR_ARRAY){
            if(tClass.isInstance(behavior))return tClass.cast(behavior);
        }

       throw new BehaviorNotFound(tClass.getName());
    }
}
