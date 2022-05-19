package org.firstinspires.ftc.teamcode.Navigation.OpenCV;

import org.opencv.core.Scalar;

public class OpenCVColors {
    //contains the HSV values for various colors

    public final static Scalar WhiteH = new Scalar(255,255,255);
    public final static Scalar WhiteL = new Scalar(230,230,230);

    public final static Scalar MarsGreenH = new Scalar(75,255,255);
    public final static Scalar MarsGreenL = new Scalar(35,58,121);

    public final static Scalar YellowH = new Scalar(30,255,255);
    public final static Scalar YellowL = new Scalar(0,100,100);

    public final static Scalar StrictYellowH = new Scalar(30,255,255);
    public final static Scalar StrictYellowL = new Scalar(20,100,100);

    public final static Scalar RedH = new Scalar(12,255,167);
    public final static Scalar RedL = new Scalar(0,80,71);

    public final static Scalar BlueH = new Scalar(188,255,189);
    public final static Scalar BlueL = new Scalar(103,101,47);

    public static Scalar[] broaden(Scalar l,Scalar h)
    {
        Scalar[] val = new Scalar[2];
        int hr = (int)h.val[0]+20;
        int hg = (int)h.val[1]+20;
        int hb = (int)h.val[2]+20;
        if(hr>255)hr=255;
        if(hg>255)hg=255;
        if(hb>255)hb=255;
        int lr = (int)l.val[0]-20;
        int lg = (int)l.val[1]-20;
        int lb = (int)l.val[2]-20;
        if(lr<0)lr=0;
        if(lg<0)lg=0;
        if(lb<0)lb=0;
        val[0]=new Scalar(lr,lg,lb);
        val[1]=new Scalar(hr,hg,hb);
        return val;
    }

    public static Scalar[] tighten(Scalar l,Scalar h)
    {
        Scalar[] val = new Scalar[2];
        int hr = (int)h.val[0]-20;
        int hg = (int)h.val[1]-20;
        int hb = (int)h.val[2]-20;
        int lr = (int)l.val[0]+20;
        int lg = (int)l.val[1]+20;
        int lb = (int)l.val[2]+20;
        if(lr>255)lr=254;
        if(lg>255)lg=254;
        if(lb>255)lb=254;
        if(hr<lr)hr=lr+1;
        if(hg<lg)hg=lg+1;
        if(hb<lb)hb=lb+1;
        if(hr<0)hr=1;
        if(hg<0)hg=1;
        if(hb<0)hb=1;
        val[0]=new Scalar(lr,lg,lb);
        val[1]=new Scalar(hr,hg,hb);
        return val;
    }
}
