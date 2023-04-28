package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private static final ArrayList<Behavior> BEHAVIOR_ARRAY = new ArrayList<>();

    /**
     * Initializes all the specified Behaviors and calls the {@link #init()} on each behavior.
     * Call this in your OpMode's {@link OpMode#init()}.
     */
    public static void systemInit(OpMode opmode, Behavior... behaviors){
        BEHAVIOR_ARRAY.clear();

        BEHAVIOR_ARRAY.addAll(Arrays.asList(behaviors));

        opMode = opmode;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.init();
            }
        }catch(Exception error){
            RobotLog.e(error.toString());
            opmode.stop();
        }
    }

    /**
     * Calls the {@linn #start()} on each behavior.
     * Call this in your OpMode's {@link OpMode#start()}.
     */
    public static void systemStart() {
        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.start();
            }
        }catch(Exception error){
            RobotLog.e(error.toString());
            opMode.stop();
        }
    }

    /**
     * Calls the {@link #update()} on each behavior.
     * Call this in your OpMode's {@link OpMode#loop()},
     * or in a while loop inside {@link LinearOpMode#runOpMode()}.
     */
    public static void systemUpdate() {
        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.update();
            }
        }catch(Exception error){
            RobotLog.e(error.toString());
            opMode.stop();
        }
    }

    /**
     * Calls the {@link #stop()} on each behavior.
     * Call this in your OpMode's {@link OpMode#stop()}.
     */
    public static void systemStop() {
        try {
            for (Behavior behavior : BEHAVIOR_ARRAY) {
                behavior.stop();
            }
        }catch(Exception error){
            RobotLog.e(error.toString());
            opMode.stop();
        }
    }

    protected static OpMode opMode;
    protected static Telemetry telemetry;
    protected static HardwareMap hardwareMap;

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
    protected static <T> T[] getHardwareArray(Class<T> tClass, String... deviceNames) throws MissingHardwareDevices {
        ArrayList<T> array = new ArrayList<T>();

        ArrayList<String> missingNames = new ArrayList<String>();

        for (String name : deviceNames) {
            T result = hardwareMap.get(tClass, name);
            if(result == null)missingNames.add(name);
            array.add(result);
        }

        if(!missingNames.isEmpty()){
            throw new MissingHardwareDevices(missingNames);
        }

        return (T[]) array.toArray();
    }

    /**
     *
     * @param tClass Hardware class as a variable.
     * @return T if found
     * @param <T> Expected Hardware Type. Should be computed by Java typing system.
     */
    static protected <T extends Behavior> T getBehavior(Class<T> tClass){
        for (Behavior behavior : BEHAVIOR_ARRAY){
            if(tClass.isInstance(behavior))return tClass.cast(behavior);
        }

        return null;
    }
}
