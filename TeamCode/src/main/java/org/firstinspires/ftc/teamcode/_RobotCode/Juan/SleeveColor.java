package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import org.opencv.core.Scalar;

import java.util.Comparator;

public enum SleeveColor {
    GREEN (105,217,181,62, 69,0  ),
    ORANGE(21, 184,219,0,  82,154),
    PURPLE(149,165,200,110,88,143);

    public final Scalar highColor;
    public final Scalar lowColor;

    static final Comparator<SleeveColor> comparator = new Comparator<SleeveColor>(){
        @Override
        public int compare(SleeveColor o1, SleeveColor o2) {
            return 0;
        }
    };

    public double certainty = 0;

    SleeveColor(int a, int b, int c, int d, int e, int f){
        highColor = new Scalar(a, b, c);
        lowColor = new Scalar(d, e, f);
    }
}