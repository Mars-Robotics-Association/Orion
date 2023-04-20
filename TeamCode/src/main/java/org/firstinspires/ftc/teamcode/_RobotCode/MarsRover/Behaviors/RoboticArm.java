package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

public class RoboticArm extends Behavior {
    // Is this overkill? I mean, Santi can program this for sure, but the maths are a bit tricky.
    // Never-mind, I got this 😎

    private static class ServoLayout{
        private ServoLayout(Servo ...servos){
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
    // 0pos = -135
    // 0pos = +135

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

    @Override
    protected void init() {
        layout = new ServoLayout(
                hardwareMap.servo.get("S2"),
                hardwareMap.servo.get("S3"),
                hardwareMap.servo.get("S4"),
                hardwareMap.servo.get("S5"),
                hardwareMap.servo.get("S6")
        );
    }

    @Override
    protected void start() {

    }

    @Override
    protected void update() {

    }

    @Override
    protected void stop() {

    }
}
