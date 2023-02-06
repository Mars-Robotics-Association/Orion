package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;

abstract class IngenuityAutonomous extends LinearOpMode {
    public static double speed = 0.6;
    IngenuityPowerPlayBot robot;
    IngenuityPowerPlayBot.SignalColor signalZone = IngenuityPowerPlayBot.SignalColor.GREEN;

    protected abstract void posMedJunction();

    protected abstract void posPreStack();

    protected abstract void posStack();

    protected abstract void posPostStack();

    protected abstract void posHighJunction();

    protected abstract void posPostHighJunction();

    protected abstract void posLowJunction();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityPowerPlayBot(this, true, true, true);
        robot.init();
        robot.getChassis().setHeadlessMode(true);

        waitForStart();
        robot.start();

        EncoderActuator arm = robot.getPayload().getArm();

        robot.ensureGripperClosed();
        sleep(500);
        robot.moveArmToStop(2);

        while (arm.getPosition() < 0.1) {
            robot.update();
            telemetry.update();
        }

        //go forward to the signal
        goToPose(22, 0, 0, 2000, true);
        signalZone = robot.readSignal();
        sleep(50);

        // try to score on the medium junction
        posMedJunction();
        dunkCone(arm);

        // get lined up for cone stack
        posPreStack();

        pickUpConeFromStack(arm, 0.058);

        scoreOnHighJunction(arm);

        if (this.time < 20) {
            pickUpConeFromStack(arm, 0.053);

            if (this.time < 18 || signalZone == IngenuityPowerPlayBot.SignalColor.RED) { // try high junction again if there's time
                scoreOnHighJunction(arm);
            } else { // score on the low junction
                robot.moveArmToStop(1);
                posLowJunction();
                dunkCone(arm);
                robot.moveArmToStop(3);
            }
        }

        switch (signalZone) {
            case RED:
                goToPose(50, 0, -175, 3000, true);
                break;
            case GREEN:
                goToPose(50, 20, -178, 3000, true);
                break;
            default:
                goToPose(48, -20, -179, 3000, true);
        }

        sleep(200);

        robot.moveArmToStop(0);
        robot.ensureGripperOpen();

        while (!isStopRequested()) {
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.update();
        }

        robot.stop();
    }

    private void scoreOnHighJunction(EncoderActuator arm) {
        robot.moveArmToStop(3);
        posHighJunction();
        dunkCone(arm);
        // straight back from high junction
        posPostHighJunction();
    }

    private void pickUpConeFromStack(EncoderActuator arm, double pos) {
        //go to cone stack
        arm.goToPosition(pos);
        sleep(100);
        posStack();
        sleep(250);

        // grab cone from stack, raise arm, and back up a little
        robot.ensureGripperClosed();
        sleep(400);
        robot.moveArmToStop(2);
        sleep(100);
        posPostStack();
    }

    private void dunkCone(EncoderActuator arm) {
        double armStartPos = arm.getPosition();
        arm.goToPosition(armStartPos - .03);
        sleep(200);
        robot.ensureGripperOpen();
        sleep(325);
        arm.goToPosition(armStartPos);
        sleep(400);
    }

    protected void goToPose(double x, double y, double angle, int bailTime, boolean stopAtEnd) {
        double started = this.time;
        IngenuityNavigation nav = robot.getNavigator();
        while (nav.goTowardsPose(x, y, angle, speed) && !isStopRequested()) {
            if ((this.time - started) * 1000 > bailTime) break;
            robot.update();
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.addData("Going to to ", "(" + x + ", " + y + ", " + angle + ")");
            telemetry.update();

        }
        if (stopAtEnd) {
            robot.getChassis().stop();
        }
    }

    private void turn(double angle) {
        while (robot.getNavigator().turnTowards(angle, speed) && !isStopRequested()) {
            robot.update();
            telemetry.addData("SIGNAL READ: ", signalZone);
            telemetry.update();
        }
    }

    private void printTelemetry() {
        /*
        //CONTROLS

        telemetry.addLine("----CONTROLS----");
        telemetry.addData("Drive with: ", "LJS");
        telemetry.addData("Turn with: ", "RJS");
        telemetry.addData("Change speed multiplier: ", "A");
        telemetry.addData("Reset robot pose: ", "Press RJS");
        telemetry.addData("Toggle headless mode: ", "Press LJS");
        telemetry.addData("Intake: ", "Hold LT");
        telemetry.addData("Load: ", "Hold RT");
        telemetry.addData("Toggle shooter: ", "Press Y");
        telemetry.addData("Toggle intake: ", "Press RB");
        telemetry.addData("Toggle path: ", "Press LB");

        robot.getPayload().printTelemetry();
        */
        //DATA
        telemetry.addLine();
        telemetry.addData("Zone: ", signalZone);
        telemetry.addLine("----DATA----");
        telemetry.addData("Gripper: ", robot.servoTarget);
        //telemetry.addData("Arm:     ", armMotor.getCurrentPosition());
        //Dead wheel positions
        telemetry.addLine("Dead wheel positions");
        double[] deadWheelPositions = robot.getNavigator().getDeadWheelPositions();
        telemetry.addData("LEFT dead wheel:       ", deadWheelPositions[0] + " inches");
        telemetry.addData("RIGHT dead wheel:      ", deadWheelPositions[1] + " inches");
        telemetry.addData("HORIZONTAL dead wheel: ", deadWheelPositions[2] + " inches");
        //Odometry estimated pose
        telemetry.addLine();
        telemetry.addLine("Robot pose");
        Pose2d robotPose = robot.getNavigator().getMeasuredPose();
        telemetry.addData("X, Y, Angle",
                Math.round(robotPose.getX() * 200) / 200
                        + ", " + Math.round(robotPose.getY() * 200) / 200 + ", "
                        + Math.round(Math.toDegrees(robotPose.getHeading()) * 200) / 200);
        telemetry.addLine();
        telemetry.update();
    }


}
