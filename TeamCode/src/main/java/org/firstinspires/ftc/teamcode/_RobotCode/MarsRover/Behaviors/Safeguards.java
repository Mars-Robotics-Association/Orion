package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Safety features intended to keep the robot from physically breaking itself.
 */
public class Safeguards extends Behavior {
    private TouchSensor[] switches;

    /**
     * {@inheritDoc}. This assumes all touch sensors are limit switches.
     */
    @Override
    protected void init() throws Exception {
        List<TouchSensor> list = new ArrayList<>();
        for (TouchSensor touchSensor : hardwareMap.touchSensor) list.add(touchSensor);

        switches = (TouchSensor[]) list.toArray();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void start() {

    }

    /**
     * {@inheritDoc}. Stops if any limit switches are activated
     */
    @Override
    protected void update() throws Exception {
        for (TouchSensor sensor : switches) {
            if(sensor.isPressed())throw new Exception("Safety Limit Switch Activated");
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void stop() {

    }
}
