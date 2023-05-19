package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain.DriveUnit;

//Motors:

//      Front Left|Front Right
//                +
//         ^M1    |    M4^
//                |
//         ^M2    |    M5^
//                |
//         ^M3    |    M6^
//                -
//       Back Left|Back Right

//Servos:

//      Front Left|Front Right
//
//        - S1 +  |  - S4 +
//                |
//          XX    |    XX
//                |
//        - S3 +  |  - S6 +
//
//       Back Left|Back Right

public class MarsDrivetrainConfig {
    public static DriveUnit[] getConfig(){
        return new DriveUnit[]{new DriveUnit(
                "M1",
                "S1",
                -10,
                +10.78
        ), new DriveUnit(
                "M2",
                null,
                -7,
                0
        ), new DriveUnit(
                "M3",
                "S3",
                -10,
                -12
        ), new DriveUnit(
                "M4",
                "S4",
                +10,
                +10.78
        ), new DriveUnit(
                "M5",
                null,
                +7,
                0
        ), new DriveUnit(
                "M6",
                "S6",
                +10,
                -12
        )};
    };
}
