package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;

//Motors:

//      Front Left|Front Right
//                +
//         ^M0    |    M3^
//                |
//         ^M1    |    M4^
//                |
//         ^M2    |    M5^
//                -
//       Back Left|Back Right

//Servos:

//      Front Left|Front Right
//
//        + S0 -  |  - S2 +
//                |
//          XX    |    XX
//                |
//        + S1 -  |  - S3 +
//
//       Back Left|Back Right

public class MarsDrivetrainConfig implements RoverDrivetrain.Configurator {

    @Override
    public void applySettings(RoverDrivetrain drivetrain) {
        drivetrain.addWheel(
                "M1",
                "S1",
                -5,
                +5
        ).addWheel(
                "M2",
                null,
                -7,
                0
        ).addWheel(
                "M3",
                "S2",
                -5,
                -5
        ).addWheel(

                "M4",
                "S3",
                +5,
                +5
        ).addWheel(
                "M5",
                null,
                +7,
                0
        ).addWheel(
                "M6",
                "S4",
                +5,
                -5
        );
    }
}
