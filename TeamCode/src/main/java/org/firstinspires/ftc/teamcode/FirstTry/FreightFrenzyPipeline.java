package org.firstinspires.ftc.teamcode.FirstTry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class FreightFrenzyPipeline extends OpenCvPipeline{

    Mat YCrCb = new Mat();
    int position;

    //these values are for red side carousel
    int subMatWidth = 30;
    int subMatHeight = 40;

    //row, column (y, x)
    int[] subMatZeroAnchor = {110,110};
    int[] subMatOneAnchor = {110,355};
    int[] subMatTwoAnchor = {110,600};

    public FreightFrenzyPipeline(int width, int height, int[] anchorZero, int[] anchorOne, int[] anchorTwo){
        //constructor
        subMatWidth = width;
        subMatHeight = height;

        subMatZeroAnchor = anchorZero;
        subMatOneAnchor = anchorOne;
        subMatTwoAnchor = anchorTwo;
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(YCrCb);

        if(YCrCb.empty()){
            return input;
        }

        Imgproc.cvtColor(YCrCb, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        //make these positions based on variables based on where auto is starting
        Mat matLeft = YCrCb.submat(subMatZeroAnchor[0], subMatZeroAnchor[0] + subMatHeight,subMatZeroAnchor[1], subMatZeroAnchor[1] + subMatWidth);
        Mat matCenter = YCrCb.submat(subMatOneAnchor[0], subMatOneAnchor[0] + subMatHeight,subMatOneAnchor[1], subMatOneAnchor[1] + subMatWidth);
        Mat matRight = YCrCb.submat(subMatTwoAnchor[0], subMatTwoAnchor[0] + subMatHeight,subMatTwoAnchor[1], subMatTwoAnchor[1] + subMatWidth);

        Imgproc.rectangle(YCrCb, new Rect(subMatZeroAnchor[1], subMatZeroAnchor[0], subMatWidth, subMatHeight), new Scalar(0,255,0));
        Imgproc.rectangle(YCrCb, new Rect(subMatOneAnchor[1], subMatOneAnchor[0], subMatWidth, subMatHeight), new Scalar(0,255,0));
        Imgproc.rectangle(YCrCb, new Rect(subMatTwoAnchor[1], subMatTwoAnchor[0], subMatWidth, subMatHeight), new Scalar(0,255,0));

        //val[1] to get first value in YCrCb, which is Cr value
        double leftTotal = Core.sumElems(matLeft).val[1];
        double centerTotal = Core.sumElems(matCenter).val[1];
        double rightTotal = Core.sumElems(matRight).val[1];

        if(leftTotal > centerTotal & leftTotal > rightTotal){
            //marker is on the left
            position = 0;
        }
        if(centerTotal > leftTotal & centerTotal > rightTotal){
            //marker is on the center
            position = 1;
        }
        if(rightTotal > leftTotal & rightTotal > centerTotal){
            //marker is on the right
            position = 2;
        }

        YCrCb.release(); //don't leak memory?
        matLeft.release();
        matCenter.release();
        matRight.release();

        return input;
        //return YCrCb;
    }
}
