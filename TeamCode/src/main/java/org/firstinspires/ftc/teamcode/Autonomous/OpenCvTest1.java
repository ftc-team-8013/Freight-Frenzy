package org.firstinspires.ftc.teamcode.Autonomous;

//IMPORTS

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Test1", group="Test")
public class OpenCvTest1 extends LinearOpMode {
    OpenCvWebcam webcam;

    //defining varibles
    BNO055IMU imu;
    Orientation angles;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor carousel;
    DcMotor crane;
    Servo arm;

    ModernRoboticsI2cRangeSensor rangeSensorM;
    ModernRoboticsI2cRangeSensor rangeSensorR;

    @Override
    public void runOpMode() throws InterruptedException {

        //init motors
        initDriveMotors();
        initMiscMotors();
        initGyro();

        //refrence TeamShippingElementDector
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Test1 detector = new Test1(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        //Starting values for varibles
        String locationOfTSE = "none";
        double barcode1 = 0;
        double barcode2 = 0;

        if (opModeIsActive()) {
            //close claw
            arm.setPosition(0);
            sleep(1500);
            //crane up out of the way
            crane.setPower(-1);
            sleep(3000);

            //determineing where the tse is
            switch (detector.getLocation()) {
                case LEFT:
                    locationOfTSE = "left";
                    break;
                case MIDDLE:
                    locationOfTSE = "middle";
                    break;
                case RIGHT:
                    locationOfTSE = "right";
                    break;
                case NOT_FOUND:
                    locationOfTSE = "not Found";
                    //if not found use gyro
                    move(0.25, 500);
                    sleep(1500);

                    barcode1 = rangeSensorM.cmUltrasonic();
                    sleep(400);

                    gyroTurning(12);
                    sleep(3000);
                    barcode2 = rangeSensorM.cmUltrasonic();
                    break;
            }


            //moveing crane to right position
            if(locationOfTSE == "right"){
                move(.25, 500);
                telemetry.addLine("Right");
                telemetry.update();
                sleep(500);
            }else if (locationOfTSE == "middle") {
                craneMotor(.5,700);
                move(.25, 500);
                telemetry.addLine("Middle");
                telemetry.update();
                sleep(500);
            }else if (locationOfTSE == "left"){
                craneMotor(.5, 1600);
                move(.25, 500);
                telemetry.addLine("Left");
                telemetry.update();
                sleep(300);
            }
            else if(locationOfTSE == "not Found"){

                sleep(400);
                telemetry.addData("one",barcode1);
                telemetry.addData("two",barcode2);
                telemetry.update();

                //for distance
                if(barcode1 <= 45 && barcode1 >= 30){
                    telemetry.addLine("Right");
                    telemetry.update();
                    sleep(500);
                }else if (barcode2 <= 50 && barcode2 >= 30) {
                    craneMotor(-5,900);
                    telemetry.addLine("Middle");
                    telemetry.update();
                    sleep(400);
                }else{
                    craneMotor(.5, 1500);
                    telemetry.addLine("Left");
                    telemetry.update();
                    sleep(300);
                }
            }
            //turning 90 degrees counterclockwise to carousel
            gyroTurning(90);

            //reverse back into carousel
            move(-0.3, 1300);

            //basic sleeping to make sure we are turning the motors as soon as the robot stops
            sleep(500);
            //turns on the carousel motor to get the duck onto the floor
            carouselMotor(1, 2000);

            //moving to warehouse
            move(0.5, 1660);

            //turning to shipping hub
            gyroTurning(0);
            sleep(500);

            //move to delivery
            move(0.25, 1300);
            sleep(750);

            //open claw
            arm.setPosition(1);
            sleep(750);

            //move back from shipping hub
            //move(-0.5, 500);

            // turn 90
            gyroTurning(90);

            //move to park
            move(1, 1250);
        }
        webcam.stopStreaming();
    }
    //Init methods
    public void initDriveMotors() {
        //Setting variables in code to a motor in the configuration.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Setting direction of motors.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initMiscMotors() {
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        crane = hardwareMap.get(DcMotor.class, "crane");
        arm = hardwareMap.get(Servo.class, "arm");

        rangeSensorM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceM");
        rangeSensorR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceR");
    }

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        sleep(250);
    }

    //Movement methods
    public boolean gyroTurning(double targetAngle) {
        boolean foundAngle = false;
        //while (opModeIsActive()) {
        while (foundAngle == false) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;

            if (angles.firstAngle >= targetAngle - 0.1 && angles.firstAngle <= targetAngle + 0.1) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                foundAngle = true;
                sleep(1000);
                break;
            } else if (angles.firstAngle >= targetAngle + 0.5) {
                if (angles.firstAngle <= targetAngle + 10) {
                    frontLeft.setPower(0.3);
                    frontRight.setPower(-0.3);
                    backLeft.setPower(0.3);
                    backRight.setPower(-0.3);
                    foundAngle = false;
                } else {
                    frontLeft.setPower(0.5);
                    frontRight.setPower(-0.5);
                    backLeft.setPower(0.5);
                    backRight.setPower(-0.5);
                    foundAngle = false;
                }
            } else if (angles.firstAngle <= targetAngle - 0.5) {
                if (angles.firstAngle >= targetAngle - 10) {
                    frontLeft.setPower(-0.3);
                    frontRight.setPower(0.3);
                    backLeft.setPower(-0.3);
                    backRight.setPower(0.3);
                    foundAngle = false;
                } else {
                    frontLeft.setPower(-0.5);
                    frontRight.setPower(0.5);
                    backLeft.setPower(-0.5);
                    backRight.setPower(0.5);
                    foundAngle = false;
                }
            }
        }
        return foundAngle;
    }


    public void stopMotors(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void move(double power, int time){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(time);
        stopMotors();
    }

    public void strafeLeft(double power, int time){
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(time);
        stopMotors();
    }

    public void strafeRight(double power, int time){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(time);
        stopMotors();
    }

    //Other methods
    public void carouselMotor(double power, int time){
        carousel.setPower(power);
        sleep(time);
        carousel.setPower(0);
    }

    public void craneMotor(double power, int time){
        crane.setPower(power);
        sleep(time);
        crane.setPower(0);
    }
}