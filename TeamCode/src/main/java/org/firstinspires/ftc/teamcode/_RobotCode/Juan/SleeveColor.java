package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import org.opencv.core.Scalar;

public enum SleeveColor {
    GREEN (105,217,181,62, 69, 0  ),
    ORANGE(21, 184,219,0,  82, 154),
    PURPLE(149,165,200,110,88, 143);

    public final Scalar highColor;
    public final Scalar lowColor;

    public double certainty = 0;

    SleeveColor(float a, float b, float c, float d, float e, float f){
        highColor = new Scalar(a, b, c);
        lowColor  = new Scalar(d, e, f);
    }
}