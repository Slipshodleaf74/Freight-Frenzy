package org.firstinspires.ftc.teamcode.FirstTry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MarkerDetector extends OpenCvPipeline {
    public final Scalar BLUE = new Scalar(0.0D, 0.0D, 255.0D);
    public final Scalar GREEN = new Scalar(0.0D, 255.0D, 0.0D);
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(85.0D, 80.0D);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(310.0D, 80.0D);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(555.0D, 80.0D);
    static final int REGION_WIDTH = 30;
    static final int REGION_HEIGHT = 40;
    Point region1_pointA;
    Point region1_pointB;
    Point region2_pointA;
    Point region2_pointB;
    Point region3_pointA;
    Point region3_pointB;
    Mat region1_Cr;
    Mat region2_Cr;
    Mat region3_Cr;
    Mat YCrCb;
    Mat Cr;
    int avg1;
    int avg2;
    int avg3;
    public volatile MarkerDetector.CappingPosition position;
    private Telemetry telemetry;

    public MarkerDetector(Telemetry telemetry) {
        this.region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        this.region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + 30.0D, REGION1_TOPLEFT_ANCHOR_POINT.y + 40.0D);
        this.region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        this.region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + 30.0D, REGION2_TOPLEFT_ANCHOR_POINT.y + 40.0D);
        this.region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        this.region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + 30.0D, REGION3_TOPLEFT_ANCHOR_POINT.y + 40.0D);
        this.YCrCb = new Mat();
        this.Cr = new Mat();
        this.position = MarkerDetector.CappingPosition.LEFT;
        this.telemetry = telemetry;
    }

    void inputToCr(Mat input) {
        Imgproc.cvtColor(input, this.YCrCb, 37);
        Core.extractChannel(this.YCrCb, this.Cr, 1);
    }

    public void init(Mat firstFrame) {
        this.inputToCr(firstFrame);
        this.region1_Cr = this.Cr.submat(new Rect(this.region1_pointA, this.region1_pointB));
        this.region2_Cr = this.Cr.submat(new Rect(this.region2_pointA, this.region2_pointB));
        this.region3_Cr = this.Cr.submat(new Rect(this.region3_pointA, this.region3_pointB));
    }

    public Mat processFrame(Mat input) {
        this.inputToCr(input);
        this.avg1 = (int)Core.mean(this.region1_Cr).val[0];
        this.avg2 = (int)Core.mean(this.region2_Cr).val[0];
        this.avg3 = (int)Core.mean(this.region3_Cr).val[0];
        Imgproc.rectangle(input, this.region1_pointA, this.region1_pointB, this.BLUE, 2);
        Imgproc.rectangle(input, this.region2_pointA, this.region2_pointB, this.BLUE, 2);
        Imgproc.rectangle(input, this.region3_pointA, this.region3_pointB, this.BLUE, 2);
        int maxOneTwo = Math.max(this.avg1, this.avg2);
        int max = Math.max(maxOneTwo, this.avg3);
        if (max == this.avg1) {
            this.position = MarkerDetector.CappingPosition.LEFT;
            Imgproc.rectangle(input, this.region1_pointA, this.region1_pointB, this.GREEN, -1);
        } else if (max == this.avg2) {
            this.position = MarkerDetector.CappingPosition.CENTER;
            Imgproc.rectangle(input, this.region2_pointA, this.region2_pointB, this.GREEN, -1);
        } else if (max == this.avg3) {
            this.position = MarkerDetector.CappingPosition.RIGHT;
            Imgproc.rectangle(input, this.region3_pointA, this.region3_pointB, this.GREEN, -1);
        }

        this.telemetry.addData("[Pattern]", this.position);
        this.telemetry.addData("Left: ", this.avg1);
        this.telemetry.addData("Middle: ", this.avg2);
        this.telemetry.addData("Right: ", this.avg3);
        this.telemetry.update();
        return input;
    }

    public MarkerDetector.CappingPosition getAnalysis() {
        return this.position;
    }

    public static enum CappingPosition {
        LEFT,
        CENTER,
        RIGHT;

        private CappingPosition() {
        }
    }
}
