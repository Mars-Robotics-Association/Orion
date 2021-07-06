package org.firstinspires.ftc.teamcode.MechanicalControl;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorizedIntake
{
    ////DEPENDENCIES////
    //Motors
    private DcMotor[] intakeMotors;
    private double[] speedMultipiers;

    public void Init(DcMotor[] setIntakeMotors, double[] setSpeedMultipliers){
        intakeMotors = setIntakeMotors;
        speedMultipiers = setSpeedMultipliers;
    }

    public void SetIntakePower(double power){
        int i = 0;
        for (DcMotor motor: intakeMotors) {
            motor.setPower(power * speedMultipiers[i]);
            i++;
        }
    }
}
