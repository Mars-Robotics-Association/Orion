package org.firstinspires.ftc.teamcode._RobotCode.MarsRoverV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//The name in quotation marks will appear on the driver station screen
@TeleOp(name = "MarsRoverTeleopv6Santi", group = "Competition")

public class MarsRoverTeleopv6 extends LinearOpMode {
	private final double CENTER_TO_FRONT_LEN        = 5; // nonzero
	private final double CENTER_TO_FRONT_HALF_WIDTH = 5; // nonzero
	private final double CENTER_TO_BACK_LEN         = 5; // nonzero
	private final double CENTER_TO_BACK_HALF_WIDTH  = 5; // nonzero

	DcMotor motorFrontLeft;
	DcMotor motorBackLeft;
	DcMotor motorFrontRight;
	DcMotor motorBackRight;
	DcMotor motorMiddleRight;
	DcMotor motorMiddleLeft;

	/**
	 * Declare the servos (use more reasonable names)
	 * https://en.wikipedia.org/wiki/Aircraft_principal_axes
	 */
	Servo servoCameraPitch;
	Servo servoCameraYaw;

	Servo servoFrontLeft;
	Servo servoFrontRight;
	Servo servoBackLeft;
	Servo servoBackRight;

	/**
	 * Distance between (x, y) and the origin (0, 0)
	 * @param y
	 * @param x
	 * @return
	 */
	private double distance(double y, double x) {
		return Math.sqrt((x * x) + (y * y));
	}

	/**
	 * Maximum number of an array of numbers.
	 * 
	 * @param arr numbers.
	 * @return
	 */
	private double maxNum(double[] arr) {
		double acc = arr[0];
		for (int i = 1; i < arr.length; i++) {
			acc = Math.max(acc, arr[i]);
		}
		return acc;
	}

	/**
	 * Cuts all motors and throws an {@link InterruptedException}
	 * 
	 * @throws InterruptedException
	 */
	private void killSystem() throws InterruptedException {
		motorFrontRight.setPower(0);
		motorMiddleRight.setPower(0);
		motorBackRight.setPower(0);

		motorFrontLeft.setPower(0);
		motorMiddleLeft.setPower(0);
		motorBackLeft.setPower(0);

		throw new InterruptedException("Gamepad kill switch triggered (gamepad2.b).");
	}

	/**
	 * Biblically-accurate Arduino map function adapted to doubles.
	 * This is pretty much f(x) where domain is [in_min, in_max] and range is
	 * [out_min, out_max].
	 * Undefined behavior when x is outside domain.
	 * @param x value to be mapped.
	 * @param in_min
	 * @param in_max
	 * @param out_min
	 * @param out_max
	 * @return
	 */
	private double map(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	@Override
	public void runOpMode() throws InterruptedException {
		// Declare the motors
		// Make sure your ID's match your configuration
		motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
		motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
		motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
		motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
		motorMiddleRight = hardwareMap.dcMotor.get("motorMiddleRight");
		motorMiddleLeft = hardwareMap.dcMotor.get("motorMiddleLeft");

		/**
		 * Declare the servos (use more reasonable names)
		 * https://en.wikipedia.org/wiki/Aircraft_principal_axes
		 */
		servoCameraPitch = hardwareMap.servo.get("servoTop");
		servoCameraYaw = hardwareMap.servo.get("servoBottom");

		servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
		servoFrontRight = hardwareMap.servo.get("servoFrontRight");
		servoBackLeft = hardwareMap.servo.get("servoBackLeft");
		servoBackRight = hardwareMap.servo.get("servoBackRight");

		// Declare refrence constant

		// 6/4/2023 changed motor speed from .5 to .25 bkf
		double motorSpeed = 0.25;

		double motorJoystick = 0;

		double FrontLeftAdjustment = 1; // remember that GoBilda servos interpret [-1.0, +1.0] as [-150deg, +150deg]

		double ServoPosition = 0.5;

		// Reverse the right side motors
		// Reverse left motors if you are using NeveRests
		// motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		// motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

		waitForStart();

		if (isStopRequested())
			return;

		// while (opModeIsActive()) {

		// /\/\ that is a big source of errors and is unecessary since the opmode:
		// (1) can throw java.lang.InterruptedException and catch it correctly from
		// outside.
		// (2) uses a java.lang.Thread to run all our code.

		// I know, I know, but I still did this to be safe and keep the code clean.
		// "If you're gonna do something, do it right"

		// double y = -gamepad1.left_stick_y; // Remember, this is reversed!
		// double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
		// double rx = gamepad1.right_stick_x;

		double y = gamepad1.left_stick_y;
		double x = gamepad1.left_stick_x;
		double rx = gamepad1.right_stick_x;

		/*
		 * Denominator is the largest motor power (absolute value) or 1
		 * This ensures all the powers maintain the same ratio
		 */

		// double denominator = maxNum(Math.abs(y), Math.abs(x), Math.abs(rx), 1);
		// double frontLeftPower = (y + x + rx) / denominator;
		// double middleLeftPower = 1;
		// double backLeftPower = (y - x + rx) / denominator;
		// double frontRightPower = (y - x - rx) / denominator;
		// double middleRightPower = 1;
		// double backRightPower = (y + x - rx) / denominator;

		// motorFrontLeft.setPower(frontLeftPower);
		// motorFrontRight.setPower(frontRightPower);
		// motorMiddleLeft.setPower(middleLeftPower);
		// motorMiddleRight.setPower(middleRightPower);
		// motorBackLeft.setPower(backLeftPower);
		// motorBackRight.setPower(backRightPower);

		if (gamepad2.a) {
			// test motor and encoder
			telemetry.addLine("Testing Motors");
			telemetry.update();

			motorFrontRight.setPower(motorSpeed);
			motorMiddleRight.setPower(motorSpeed);
			motorBackRight.setPower(motorSpeed);

			motorFrontLeft.setPower(-motorSpeed);
			motorMiddleLeft.setPower(-motorSpeed);
			motorBackLeft.setPower(-motorSpeed);

			sleep(5000);

			motorFrontRight.setPower(0);
			motorMiddleRight.setPower(0);
			motorBackRight.setPower(0);

			motorFrontLeft.setPower(0);
			motorMiddleLeft.setPower(0);
			motorBackLeft.setPower(0);

			telemetry.addLine("Motor test complete, encoder values below:");
			telemetry.addData("FrontLeft:", motorFrontLeft.getCurrentPosition());
			telemetry.addData("MiddleLeft:", motorMiddleLeft.getCurrentPosition());
			telemetry.addData("BackLeft:", motorBackLeft.getCurrentPosition());

			telemetry.addData("FrontRight:", motorFrontRight.getCurrentPosition());
			telemetry.addData("MiddleRight:", motorMiddleRight.getCurrentPosition());
			telemetry.addData("BackRight:", motorBackRight.getCurrentPosition());

			telemetry.addLine("Servo Test Beginning");

			telemetry.update();

			sleep(2000);

			servoFrontLeft.setPosition(0.5);
			servoFrontRight.setPosition(0.5);
			servoBackLeft.setPosition(0.5);
			servoBackRight.setPosition(0.5);

			sleep(1000);

			servoFrontLeft.setPosition(0.3);
			servoFrontRight.setPosition(0.3);
			servoBackLeft.setPosition(0.3);
			servoBackRight.setPosition(0.3);

			sleep(1000);

			servoFrontLeft.setPosition(0.5);
			servoFrontRight.setPosition(0.5);
			servoBackLeft.setPosition(0.5);
			servoBackRight.setPosition(0.5);

			sleep(1000);

			servoFrontLeft.setPosition(0.7);
			servoFrontRight.setPosition(0.7);
			servoBackLeft.setPosition(0.7);
			servoBackRight.setPosition(0.7);

			sleep(1000);

			servoFrontLeft.setPosition(0.5);
			servoFrontRight.setPosition(0.5);
			servoBackLeft.setPosition(0.5);
			servoBackRight.setPosition(0.5);

			sleep(1000);

			telemetry.addLine("Servo Test Conclueded");
			telemetry.update();
		}

		// start kill process if button b on gamepad2 is pressed
		if (gamepad2.b)
			killSystem();

		while (opModeIsActive()) {
			if (gamepad2.b)
				killSystem();

			// this should be running on each "update"

			// just realized most of this code can be converted straight to
			// SIMD (using OpenCV), but the performance gain is negligible.
			// (I'll try to write some WAT machine code to challenge myself after this)

			double dirMult = Math.copySign(1, gamepad1.left_stick_x); // +1 for right, -1 for left
			double speedMult = gamepad1.left_stick_y; // +real for forward, -real for backwrd

			// should be updated by "map" function. positive nonzero
			double turnRadius = map(Math.abs(gamepad1.left_stick_x), 0, 1, 6, 48);
			

			double CF_LEN        = CENTER_TO_FRONT_LEN       ;
			double CF_HALF_WIDTH = CENTER_TO_FRONT_HALF_WIDTH;
			double CB_LEN        = CENTER_TO_BACK_LEN        ;
			double CB_HALF_WIDTH = CENTER_TO_BACK_HALF_WIDTH ;

			// Remember the formulas.
			double frontLeftRad  = Math.tan(CF_LEN / (turnRadius + CF_HALF_WIDTH));
			double backLeftRad   = Math.tan(CB_LEN / (turnRadius + CB_HALF_WIDTH));
			double frontRightRad = Math.tan(CF_LEN / (turnRadius - CF_HALF_WIDTH));
			double backRightRad  = Math.tan(CB_LEN / (turnRadius - CB_HALF_WIDTH));

			telemetry.addData("Turn radius", turnRadius);
			telemetry.addData("Front left degrees", Math.toDegrees(frontLeftRad));
			telemetry.addData("Back left degrees", Math.toDegrees(backLeftRad));
			telemetry.addData("Front right degrees", Math.toDegrees(frontRightRad));
			telemetry.addData("Back right degrees", Math.toDegrees(backRightRad));

			// map from domain [0.0, 300.0] degrees to range [0.0, 1.0]
			// or [0.0, 150.0] to [0.0, 0.5]
			double TO_POS = Math.PI * (5 / 3) * dirMult;

			// gotta 
			frontLeftRad  /= TO_POS * +1;
			backLeftRad   /= TO_POS * +1;
			frontRightRad /= TO_POS * -1;
			backRightRad  /= TO_POS * -1;

			servoFrontLeft .setPosition(.5 + frontLeftRad );
			servoBackLeft  .setPosition(.5 + backLeftRad  );
			servoFrontRight.setPosition(.5 + frontRightRad);
			servoBackRight .setPosition(.5 + backRightRad );

			double frontLeftPower   = distance(CF_LEN, (turnRadius + CF_HALF_WIDTH * dirMult));
			double middleLeftPower  = 1; // center wheel
			double backLeftPower    = distance(CB_LEN, (turnRadius + CB_HALF_WIDTH * dirMult));
			double frontRightPower  = distance(CF_LEN, (turnRadius - CF_HALF_WIDTH * dirMult));
			double middleRightPower = 1; // center wheel
			double backRightPower   = distance(CB_LEN, (turnRadius - CB_HALF_WIDTH * dirMult));

			frontLeftPower   *= -speedMult;
			middleLeftPower  *= -speedMult;
			backLeftPower    *= -speedMult;
			frontRightPower  *= +speedMult;
			middleRightPower *= +speedMult;
			backRightPower   *= +speedMult;

			double normalDivisor = maxNum(new double[]{
					frontLeftPower,
					middleLeftPower,
					backLeftPower,
					frontRightPower,
					middleRightPower,
					backRightPower});

			frontLeftPower   /= normalDivisor;
			middleLeftPower  /= normalDivisor;
			backLeftPower    /= normalDivisor;
			frontRightPower  /= normalDivisor;
			middleRightPower /= normalDivisor;
			backRightPower   /= normalDivisor;

			motorFrontLeft.setPower(-frontLeftPower);
			motorMiddleLeft.setPower(-middleLeftPower);
			motorBackLeft.setPower(-backLeftPower);

			motorFrontRight.setPower(frontRightPower);
			motorMiddleRight.setPower(middleRightPower);
			motorBackRight.setPower(backRightPower);

			telemetry.update();
		}
	}
}