package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode._RobotCode.Archived.MecanumBaseControl;
import org.firstinspires.ftc.teamcode.Orion.NavProfiles.NavigationProfile;

@Config
public class Andrew extends MecanumBaseControl
{
    public Andrew(OpMode setOpMode, NavigationProfile setNavProfile, HermesLog setLog, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, setNavProfile, setLog, useChassis, usePayload, useNavigator);
        super.InitCoreRobotModules();
    }

    public void Start(){
        super.StartCoreRobotModules();
    }
}
