package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Orion.Archive.NavProfiles.ChassisProfile;
import org.firstinspires.ftc.teamcode._RobotCode.Archived.MecanumBaseControl;
import org.firstinspires.ftc.teamcode.Orion.Archive.NavProfiles.NavigationProfile;

@Config
public class Andrew extends MecanumBaseControl
{
    public Andrew(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new EmptyProfile(), new HermesLog("SoccerBot", 500, setOpMode), useChassis, usePayload, useNavigator);
    }

    public void Start(){
        super.StartCoreRobotModules();
    }
}
