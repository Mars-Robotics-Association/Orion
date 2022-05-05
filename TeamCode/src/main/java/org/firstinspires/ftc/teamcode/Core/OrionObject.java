package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OrionObject
{
   protected OpMode opMode;
   protected String name = "NO_NAME";
   public String GetName() {return name;}
   protected double version = 0.0;
   public double GetVersion() {return version;}

   protected void SetOpModeNameVersion(OpMode setOpMode, String setName, double setVersion){
      opMode = setOpMode;
      name = setName;
      version = setVersion;
   }
}
