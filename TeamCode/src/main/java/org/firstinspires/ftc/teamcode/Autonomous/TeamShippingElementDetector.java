package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

public class TeamShippingElementDetector extends OpenCvPipeline{
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    private Location location;

    static final Rect MID_ROI = new Rect(
        new Point(60, 35),
        new Point(120, 75));

    static double percentThreshold = 0.4;

    public TeamShippingElementDetector(Telemetry t){
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // Yellowish
        Scalar yellowLowHSV = new Scalar(23,70,70);
        Scalar yellowHighHSV = new Scalar(32, 255, 255);

        // Orangish
        Scalar orangeLowHSV = new Scalar(0, 59, 107);
        Scalar orangeHighHSV = new Scalar(71, 183, 255);

        Core.inRange(mat, orangeLowHSV, orangeHighHSV, mat);

        Mat mid = mat.submat(MID_ROI);

        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 100;

        mid.release();

        telemetry.addData("raw Mid value", (int) Core.sumElems(mid).val[0]);
        telemetry.addData("Mid percentage", Math.round(midValue * 100) + "%");

        boolean tseMid = midValue > percentThreshold;

        if (tseMid){
            location = Location.MIDDLE;
            telemetry.addData("Location", "Middle");
        }else{
            location = Location.NOT_FOUND;
            telemetry.addData("Location", "not found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar tseColor = new Scalar(255,0,0);

        Imgproc.rectangle(mat, MID_ROI, tseColor);
        return mat;
    }
    public Location getLocation() {
        return location;
    }
}
