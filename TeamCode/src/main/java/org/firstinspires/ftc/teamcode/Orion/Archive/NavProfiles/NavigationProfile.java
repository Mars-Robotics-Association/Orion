package org.firstinspires.ftc.teamcode.Orion.Archive.NavProfiles;

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
