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
     * Thrown by {@link #getHardwareArray} when a hardware device could not be found.
     */
    static class MissingHardwareDevices extends Exception{
        MissingHardwareDevices(ArrayList<String> devices){
            super(String.join("\n",
                    "Could not find the following devices in the Hardware Map:",
                    String.join("\n", devices)
            ));
        }
    }

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

    private static boolean sendException() {
        if (!running) {
            telemetry.clearAll();

            Exception exception = currentException;
            telemetry.addLine("Orbitross crashed at " + crashedInPhase + " Phase");
            telemetry.addData("Cause", exception.getMessage());
            telemetry.addLine("Stack Trace");
            for (StackTraceElement element: exception.getStackTrace()) {
                telemetry.addLine(element.toString());
            }
            telemetry.addLine();

            telemetry.update();
        }

        return !running;
    }

    public static void sendException(Exception exception, String phase) {
        currentException = exception;
        crashedInPhase = phase;
        running = false;

        sendException();
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
            currentException = e;
            crashedInPhase = "Init";
            running = false;

            sendException();
        }
    }

    /**
     * Calls the {@link #start()} on each behavior.
     * Call this in your OpMode's {@link OpMode#start()}.
     */
    public static void systemStart() {
        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.start();
            }
        }catch(Exception e){
            currentException = e;
            crashedInPhase = "Start";
            running = false;
        }
    }

    /**
     * Calls the {@link #update()} on each behavior.
     * Call this in your OpMode's {@link OpMode#loop()},
     * or in a while loop inside {@link LinearOpMode#runOpMode()}.
     */
    public static void systemUpdate() {
        if(sendException())return;

        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.update();
            }
        }catch(Exception e){
            currentException = e;
            crashedInPhase = "Update";
            running = false;
        }
    }

    /**
     * Calls the {@link #stop()} on each behavior.
     * Call this in your OpMode's {@link OpMode#stop()}.
     */
    public static void systemStop() {
        if(sendException())return;

        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.stop();
            }
        }catch(Exception e){
            currentException = e;
            crashedInPhase = "Stop";
            running = false;

            sendException();
        }
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
     * Hardware mapping helper for getting multiple devices of the same type with less code.
     * @param <T> Device type you are looking for. Should be guessed for you by the type system.
     * @return Your requested devices.
     * @param tClass Device type's class as a value (e.g. DCMotor.class).
     * @param deviceNames List of device names. Output will have the same order of this.
     * @throws MissingHardwareDevices Will throw and list all missing hardware, if any.
     */
    protected <T> ArrayList<T> getHardwareArray(Class<T> tClass, String... deviceNames) throws MissingHardwareDevices {
        ArrayList<T> devices = new ArrayList<>(deviceNames.length);

        ArrayList<String> missingNames = new ArrayList<>(deviceNames.length);

        for (String name : deviceNames) {
            T result = hardwareMap.get(tClass, name);
            if(result == null){
                missingNames.add(name);
            }else{
                devices.add(result);
            }

        }

        if(!missingNames.isEmpty()){
            throw new MissingHardwareDevices(missingNames);
        }



        return devices;
    }

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
