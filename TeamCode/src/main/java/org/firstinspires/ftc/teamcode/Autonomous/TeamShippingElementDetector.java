package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TeamShippingElementDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();


    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }

    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(30, 10),
            new Point(60, 150));

    static final Rect MID_ROI = new Rect(
            new Point(140, 10),
            new Point(170, 150));

    static final Rect RIGHT_ROI = new Rect(
            new Point(240, 10),
            new Point(270, 150));

    static double percentThreshold = 0.1;

    public TeamShippingElementDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // Yellowish
        Scalar yellowLowHSV = new Scalar(23, 70, 70);
        Scalar yellowHighHSV = new Scalar(32, 255, 255);

        // Orangish
        Scalar orangeLowHSV = new Scalar(0, 59, 107);
        Scalar orangeHighHSV = new Scalar(71, 183, 255);

        Core.inRange(mat, orangeLowHSV, orangeHighHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat mid = mat.submat(MID_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 100;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 100;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 100;

        mid.release();

        telemetry.addData("raw Mid value", (int) Core.sumElems(mid).val[0]);
        telemetry.addData("Mid percentage", Math.round(midValue * 100) + "%");

        boolean tseLeft = leftValue > percentThreshold;
        boolean tseMid = midValue > percentThreshold;
        boolean tseRight = rightValue > percentThreshold;

        if (tseLeft) {
            location = Location.LEFT;
            telemetry.addData("Location", "Left");
        } else if (tseMid) {
            location = Location.MIDDLE;
            telemetry.addData("Location", "Middle");
        } else if (tseRight) {
            location = Location.RIGHT;
            telemetry.addData("Location", "Right");
        } else {
            location = Location.NOT_FOUND;
            telemetry.addData("Location", "not found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar tseColor = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LEFT_ROI, tseColor);
        Imgproc.rectangle(mat, MID_ROI, tseColor);
        Imgproc.rectangle(mat, RIGHT_ROI, tseColor);
        return mat;
    }

    public Location getLocation() {
        return location;
    }
    //if (opModeIsActive) {
    //closing the arm and waiting so we know the block is in possession
        arm.setPosition(0);

    sleep(1500);

        crane.setPower(-3);

    sleep(100);

    stopMotors();

    //move forwards a few inches
    move(0.25,500);

    sleep(1500);

    double barcode1 = rangeSensorM.cmUltrasonic();

    sleep(400);

    gyroTurning(12);

    sleep(3000);

    double barcode2 = rangeSensorM.cmUltrasonic();

    sleep(400);
        telemetry.addData("one",barcode1);
        telemetry.addData("two",barcode2);
        telemetry.update();

    //turning 90 degrees counterclockwise
    gyroTurning(90);

    //reverse back into carousel
    move(-0.3,1300);

    //basic sleeping to make sure we are turning the motors as soon as the robot stops
    sleep(500);

    //turns on the carousel motor to get the duck onto the floor
    carouselMotor(1,2000);
    //changed power of motor from .5

    //turning on the crane motor making the crane go up and avoid the terrain
    if(barcode1 <=45&&barcode1 >=30)

    {
        telemetry.addLine("Right");
        telemetry.update();
        sleep(500);
    }else if(barcode2 <=50&&barcode2 >=30)

    {
        craneMotor(-5, 900);
        telemetry.addLine("Middle");
        telemetry.update();
        sleep(400);
    }else

    {
        craneMotor(.5, 1500);
        telemetry.addLine("Left");
        telemetry.update();
        sleep(300);
    }

    //moving to warehouse
    move(0.5,1680);

    //turning to shipping hub
    gyroTurning(0);

    sleep(500);

    //move to delivery
    move(0.25,1450);

    sleep(750);

    //open claw
    arm.setPosition(1);

    sleep(750);

    //move back from shipping hub
    move(-0.5,500);
    //250 to 200

    // turn 90
    gyroTurning(90);

    //if (rangeSensorM.cmUltrasonic() <=50) {
    //telemetry.addLine("hi");
    //telemetry.update();
    //stopMotors();
    //}
    //else {
    //move(1,1750);
    //}
    //move to warehouse
    move(1,1750);



}
