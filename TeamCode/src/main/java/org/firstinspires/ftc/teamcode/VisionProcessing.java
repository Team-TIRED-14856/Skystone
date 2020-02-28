package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisionProcessing extends OpenCvPipeline {

    //Public stuff for access by other classes
    public int stoneX, stoneY;
    public boolean isStone = false;

    //This is called Every frame
    Mat hierarchy;
    Rect[] rects;
    ArrayList<Rect> filteredRects;
    @Override
    public Mat processFrame(Mat rgba) {

        Mat mat = rgba;
        Mat unchanged = mat.clone();
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24,24));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24,24));

        //Remove noise
        Imgproc.blur(mat, mat, new Size(7,7));

        //Convert the frame to HSV
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);

        //Get Thresholding values
        Scalar minValues = new Scalar(0,0,0);
        Scalar maxValues = new Scalar(255,255,20);

        //Threshold HSV image to select box
        Core.inRange(mat, minValues, maxValues, mat);

        Imgproc.erode(mat, mat, erodeElement);
        Imgproc.dilate(mat, mat, dilateElement);
        List<MatOfPoint> contours = new ArrayList<>();
        hierarchy = new Mat();

        //Find contours
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
        rects = new Rect[contours.size()];
        filteredRects = new ArrayList<>();

        for(int i = 0 ; i < rects.length ; i++){
            rects[i] = Imgproc.boundingRect(contours.get(i));
            Imgproc.rectangle(unchanged, new Point(rects[i].x, rects[i].y), new Point(rects[i].x+rects[i].width,rects[i].y+rects[i].height), new Scalar(200,76,175), 2);
        }

        for(int i = 0 ; i < rects.length; i++){
            if(rects[i].y > unchanged.height()/2){
                filteredRects.add(rects[i]);
            }
        }

        //Find the skystone
        if(filteredRects.size()>1){
            if(filteredRects.get(0).x < rects[1].x){
                stoneX = (filteredRects.get(0).x+filteredRects.get(0).width + filteredRects.get(1).x)/2;
                stoneY = filteredRects.get(0).y + filteredRects.get(0).height/2;
            }
            else{
                stoneX = (filteredRects.get(0).x + filteredRects.get(1).x + filteredRects.get(1).width)/2;
                stoneY = filteredRects.get(0).y + filteredRects.get(0).height/2;
            }
            Imgproc.circle(unchanged, new Point(stoneX,stoneY), 10, new Scalar(67,255,175));

            if(stoneX > 4*unchanged.width()/9 && stoneX < 5*unchanged.width()/9){
                isStone = true;
            }
            else{
                isStone = false;
            }
        }
        else{
            isStone = false;
        }

        return unchanged;
    }
}
