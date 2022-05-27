package org.firstinspires.ftc.teamcode.Navigation.OpenCV;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class OpenCV {
    //takes a Mat image and converts it to a Bitmap
    public static Bitmap convertMatToBitMap(Mat input){
        Bitmap bmp = Bitmap.createBitmap(input.width(),input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input,bmp);
        return bmp;
    }

    //takes a Bitmap image and converts it to a Mat
    public static Mat convertBitmapToMat(Bitmap input){
        Mat mat = new Mat();
        Utils.bitmapToMat(input,mat);
        return mat;
    }

    //takes a Mat and isolates the color yellow
    public static Mat IsolateYellow(Mat input){
        Scalar lowhsv = new Scalar(0,100,100);
        Scalar highhsv = new Scalar(30, 255, 255);
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowhsv,highhsv, mask);
        Core.bitwise_and(input, input, last, mask);
        return last;
    }

    //takes a Mat and isolates the color white
    public static Mat IsolateWhite(Mat input){
        Scalar highhsv = new Scalar(255,255,255);
        Scalar lowhsv = new Scalar(230,230,230);
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowhsv,highhsv, mask);
        Core.bitwise_and(input, input, last, mask);
        return last;
    }

    //takes a Mat and isolates the color blue
    public static Mat IsolateBlue(Mat input){
        Scalar highhsv = new Scalar(118,255,189);
        Scalar lowhsv = new Scalar(103,101,47);
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowhsv,highhsv, mask);
        Core.bitwise_and(input, input, last, mask);
        return last;
    }

    //takes a Mat and isolates the color red
    public static Mat IsolateRed(Mat input){
        Scalar highhsv = new Scalar(12,255,167);
        Scalar lowhsv = new Scalar(0,80,71);
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowhsv,highhsv, mask);
        Core.bitwise_and(input, input, last, mask);
        return last;
    }

    //for determining nonblack pixels in a color isolated image
    public static int countPixels(Bitmap input){
        int pixelcount = 0;

        for (int x = 0; x <input.getWidth(); x++) {
            for (int y = 0; y < input.getHeight(); y++) {
                int color = input.getPixel(x,y);
                int R = (color & 0xff0000) >> 16;
                int G = (color & 0xff00) >> 8;
                int B = color & 0xff;
                if(!((R == 0) && (G == 0) && (B == 0))){
                    pixelcount++;
                }
            }
        }
        return pixelcount;
    }

    public static double percentColor(Bitmap input)
    {
        return (double)countPixels(input)/(input.getHeight()*input.getWidth());
    }

    //returns average length and width of all colored pixels in a color isolated image
    public static int[] findColor(Bitmap input){
        int width = 0,height=0,count=0;
        for(int w = 0;w<input.getWidth();w++){
            for (int h = 0; h < input.getHeight(); h++) {
                int color = input.getPixel(w, h);
                int R = (color & 0xff0000) >> 16;
                int G = (color & 0xff00) >> 8;
                int B = color & 0xff;
                if (!((R == 0) && (G == 0) && (B == 0))) {
                    width += w;
                    height += h;
                    count++;
                }
            }
        }
        if(count==0){
            return new int[]{-1,-1};
        }
        return new int[]{width/count,height/count};
    }

    //isolate a color from a mat
    public static Mat isolateColor(Mat input, Scalar highhsv, Scalar lowhsv){
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowhsv,highhsv, mask);
        Core.bitwise_and(input, input, last, mask);
        return last;
    }

    public static Bitmap ShrinkBitmap(Bitmap bitmapIn, int width, int height){
        return Bitmap.createScaledBitmap(bitmapIn, width, height, true); //might want to set filter to false (uses more proccessing power to make better image
    }

    public static Bitmap GrowBitmap(Bitmap input,int width, int height){
        if(width<input.getWidth()||height<input.getHeight())return input;
        Bitmap bmp = Bitmap.createBitmap(width,height,Bitmap.Config.RGB_565);
        for(int x=0;x<width;x++){
            for(int y=0;y<height;y++){
                int color = input.getPixel((x*input.getWidth())/width,(y*input.getHeight())/height);
                int R = (color & 0xff0000) >> 16;
                int G = (color & 0xff00) >> 8;
                int B = color & 0xff;
                bmp.setPixel(x,y, Color.rgb(R,G,B));
            }
        }
        return bmp;
    }

    //get extreme top bottom left and right values of a color isolated image
    public static int[] getTBLR(Bitmap input){
        int maxh = Integer.MIN_VALUE,minh=Integer.MAX_VALUE, maxw = Integer.MIN_VALUE,minw=Integer.MAX_VALUE;
        for(int w = 0;w<input.getWidth();w++){
            for (int h = 0; h < input.getHeight(); h++) {
                int color = input.getPixel(w, h);
                int R = (color & 0xff0000) >> 16;
                int G = (color & 0xff00) >> 8;
                int B = color & 0xff;
                if (!((R == 0) && (G == 0) && (B == 0))) {
                    if(h<minh)minh=h;
                    if(h>maxh)maxh=h;
                    if(w<minw)minw=w;
                    if(w>maxw)maxw=w;
                }
            }
        }
        return new int[]{minh,maxh,minw,maxw};
    }

    public static Mat crop(Mat input, Rect rect) {
        return input.submat(rect);
    }

    public static Mat range(Mat input, Scalar low, Scalar high)
    {
        Mat hsvMat = new Mat();
        Mat mask = new Mat();
        Imgproc.cvtColor(input,hsvMat,Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat,low,high,mask);
        return mask;
    }
}
