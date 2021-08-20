package org.firstinspires.ftc.teamcode._RobotCode.BelindaChassis;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Orion.NavProfiles.TuningProfile;

public class BelindaTuningProfile extends TuningProfile
{
    public static final double kV =  0.01494; //1.0 / rpmToVelocity(MAX_RPM)
    public static final double kA = 0.00004;
    public static final double kStatic = 0.04877; //0.06510
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8.0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8.0, 0, 0);
    public static double LATERAL_MULTIPLIER = 1;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public static final double MAX_VEL = 60.0;
    public static final double MAX_ACCEL = 40.0;
    public static final double MAX_ANG_VEL = 10.0;
    public static final double MAX_ANG_ACCEL = 5.0;
    public static double X_MULTIPLIER = 1.0; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0; // Multiplier in the Y direction

    @Override
    public double kV() {return kV;}

    @Override
    public double kA() {return kA;}

    @Override
    public double kStatic() {return kStatic;}

    @Override
    public PIDCoefficients TRANSLATIONAL_PID() {return TRANSLATIONAL_PID;}

    @Override
    public PIDCoefficients HEADING_PID() {return HEADING_PID;}

    @Override
    public double LATERAL_MULTIPLIER() {return LATERAL_MULTIPLIER;}

    @Override
    public double VX_WEIGHT() {return VX_WEIGHT;}

    @Override
    public double VY_WEIGHT() {return VY_WEIGHT;}

    @Override
    public double OMEGA_WEIGHT() {return OMEGA_WEIGHT;}

    @Override
    public double MAX_VEL() {return MAX_VEL;}

    @Override
    public double MAX_ACCEL() {return MAX_ACCEL;}

    @Override
    public double MAX_ANG_VEL() {return MAX_ANG_VEL;}

    @Override
    public double MAX_ANG_ACCEL() {return MAX_ANG_ACCEL;}

    @Override
    public double X_MULTIPLIER() {return X_MULTIPLIER;}

    @Override
    public double Y_MULTIPLIER() {return Y_MULTIPLIER;}
}
