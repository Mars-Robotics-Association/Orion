package org.firstinspires.ftc.teamcode._RobotCode.Archived.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

class TextCycler
{
    OpMode opMode;
    String[] text = {"I'm a HAPPY robot", "I'm a winner!", "All other robots will DIE!", "P O S I T I V I T Y !", "WEEEEEE! Lets DRIVE!", "Not just a robot- the BEST robot!"};
    String current = "AHHHHHHHHHHHHHHHHH";
    int index = 0;
    double updateTime = 5;
    double lastTime = 0;

    public TextCycler(OpMode setOpMode){opMode = setOpMode;}

    public void Update(){
        if(opMode.getRuntime() > lastTime + updateTime){
            index++;
            if(index >= text.length) index = 0;
            current = text[index];
            lastTime = opMode.getRuntime();
        }
        opMode.telemetry.addLine(current);

    }
}
