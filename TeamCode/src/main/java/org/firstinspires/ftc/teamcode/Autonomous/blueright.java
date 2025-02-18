package org.firstinspires.ftc.teamcode.Autonomous;

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


@Autonomous(name="blueright", group="Auto")
public class blueright extends LinearOpMode {
    OpenCvWebcam webcam;

    robotClass robot = new robotClass();

    ModernRoboticsI2cRangeSensor rangeSensorM;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        //refrence TeamShippingElementDector
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        TeamShippingElementDetector detector = new TeamShippingElementDetector(telemetry);
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
            robot.arm.setPosition(0);
            sleep(1500);
            //crane up out of the way
            //-800
            robot.crane.setTargetPosition(-800);
            robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.crane.setPower(-1);
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
                    robot.move(0.25, 500);
                    sleep(1500);

                    barcode1 = rangeSensorM.cmUltrasonic();
                    sleep(400);

                    robot.gyroTurning(15);
                    sleep(1000);
                    robot.gyroTurning(-90);
                    barcode2 = rangeSensorM.cmUltrasonic();
                    break;
            }


            //moving crane to right position
            if(locationOfTSE == "right"){
                robot.move(.25, 500);
                telemetry.addLine("Right");
                telemetry.update();
                sleep(500);
            }else if (locationOfTSE == "middle") {
                //-463
                robot.crane.setTargetPosition(-500);
                robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.crane.setPower(0.5);
                robot.move(.25, 500);
                telemetry.addLine("Middle");
                telemetry.update();
                sleep(500);
            }else if (locationOfTSE == "left"){
                //-214
                robot.crane.setTargetPosition(-250);
                robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.crane.setPower(.5);
                robot.move(.25, 500);
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
                    robot.crane.setTargetPosition(-463);
                    robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.crane.setPower(.5);
                    telemetry.addLine("Middle");
                    telemetry.update();
                    sleep(400);
                }else{
                    robot.crane.setTargetPosition(-214);
                    robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.crane.setPower(.5);
                    robot.craneMotor(.5, 1500);
                    telemetry.addLine("Left");
                    telemetry.update();
                    sleep(300);
                }

            }
            //turning 90 degrees counterclockwise to carousel
            robot.gyroTurning(90);

            //reverse back into carousel
            robot.move(-.3, 1000);
            //basic sleeping to make sure we are turning the motors as soon as the robot stops
            sleep(500);

            //turns on the carousel motor to get the duck onto the floor
            robot.carouselMotor(.85,2000);


            robot.move(.2,200);

            robot.gyroTurning(90);

            //moving to warehouse
            robot.move(.75,900);

            //turning to shipping hub
            robot.gyroTurning(0);
            sleep(500);

            //move to delivery
            robot.move(0.25, 1050);
            sleep(200);

            robot.gyroTurning(0);

            //open claw
            robot.arm.setPosition(1);
            sleep(700);

            //back up to wall
            robot.move(-.5,700);

            sleep(200);

            robot.crane.setTargetPosition(-300);
            robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.crane.setPower(.5);

            robot.move(.3,300);

            //turn to warehouse
            robot.gyroTurning(-90);

            sleep(500);

            //move to warehouse
            //move(-1, 1300);


            robot.frontLeft.setPower(-1);
            robot.frontRight.setPower(-.88);
            robot.backLeft.setPower(-1);
            robot.backRight.setPower(-.88);
            sleep(1250);
            robot.stopMotors();

            robot.gyroTurning(0);

            robot.move(-.5,500);



        }
        webcam.stopStreaming();
    }
}