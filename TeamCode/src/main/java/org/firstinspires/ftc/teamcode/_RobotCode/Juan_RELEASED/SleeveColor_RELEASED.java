package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import org.opencv.core.Scalar;

public enum SleeveColor_RELEASED {
    GREEN (105,217,181,62, 69,0  ),
    ORANGE(21, 184,219,0,  82,154),
    PURPLE(149,165,200,110,88,143);

    public final Scalar highColor;
    public final Scalar lowColor;

    SleeveColor_RELEASED(int a, int b, int c, int d, int e, int f){
        highColor = new Scalar(a, b, c);
        lowColor = new Scalar(d, e, f);
    }
}