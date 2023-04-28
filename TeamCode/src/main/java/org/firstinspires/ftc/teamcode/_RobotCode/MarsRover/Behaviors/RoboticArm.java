package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

import java.util.ArrayList;

public class RoboticArm extends Behavior {

    private static class ServoLayout{
        private ServoLayout(Servo[] servos){
            baseY    = servos[0];
            baseZ    = servos[1];
            boomZ    = servos[2];
            subjectZ = servos[3];
            subjectY = servos[4];
        }

        Servo baseY, baseZ, boomZ, subjectZ, subjectY;
    }

    // Note to mechanical!

    // Defaults are set to
    // 0.0 pos = -135
    // 1.0 pos = +135

    // Servos must be mapped to
    // 0pos = 0deg
    // 1pos = 270deg

    private double convert(double radians){
        return (2 * radians) / (3 * Math.PI);
    }

    private ServoLayout layout;

    final double BOOM_LENGTH = .5;

    // Think Unit Circle!

    //     [-1,+1]    |    [+1,+1]
    //                |
    //                |
    //                |
    //     [-1,+0]    X    [+1,+0]
    //                ^
    //         Attachment Point

    // You can use Trigonometry to map an object's position to this

    public void reachToPoint(double x, double y){
        // Rotate!

        layout.baseY.setPosition(
                1 - (Math.atan2(y, x) / Math.PI)
        );

        // Extend!

        double distance = Math.sqrt((x * x) + (y * y));
        double halfDist = distance / 2;
        double cosine = Math.cos(halfDist / BOOM_LENGTH);

        layout.baseZ.setPosition(convert(cosine));

        layout.subjectZ.setPosition(convert(cosine));

        // All angles in a circle add up to 180 degrees (or Pi!)
        layout.boomZ.setPosition(convert(Math.PI - cosine * 2));
    }

    public void takeSample(double x, double y){
        // ???
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void init() throws Exception {
        layout = new ServoLayout(getHardwareArray(Servo.class,
                "S4","S5","S6","S7","S8"
        ));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void start() {

    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void update() {

    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void stop() {

    }
}
