package org.firstinspires.ftc.teamcode.Orion.NavProfiles;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class NavigationProfile
{
    public TuningProfile tuningProfile;
    public ChassisProfile chassisProfile;
    public VisionProfile visionProfile;
    public OdometryProfile odometryProfile;

    public NavigationProfile(TuningProfile setTuningProfile, ChassisProfile setChassisProfile, VisionProfile setVisionProfile, OdometryProfile setOdometryProfile){
        tuningProfile = setTuningProfile;
        chassisProfile = setChassisProfile;
        visionProfile = setVisionProfile;
        odometryProfile = setOdometryProfile;
    }
}
