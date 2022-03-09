package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotClass;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="redright", group="Auto")
public class redRight extends LinearOpMode {
    robotClass robot = new robotClass();

    public OpenCvWebcam webcam = null;

    @Override
    public void runOpMode() throws InterruptedException {

        //init motors
        robot.init(hardwareMap);

        //refrence TeamShippingElementDector
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
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
            telemetry.addData("Ecoder value", robot.crane.getCurrentPosition());
            telemetry.update();
            //close claw
            robot.arm.setPosition(0);
            sleep(1500);
            //crane up out of the way
            robot.crane.setTargetPosition(-900);
            robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.crane.setPower(-1);
            telemetry.addData("Ecoder value", robot.crane.getCurrentPosition());
            telemetry.update();
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

                    barcode1 = robot.rangeSensorM.cmUltrasonic();
                    sleep(400);

                    robot.gyroTurning(12, 500L);
                    sleep(3000);
                    barcode2 = robot.rangeSensorM.cmUltrasonic();
                    break;
            }


            //moveing crane to right position
            if (locationOfTSE == "right") {
                robot.move(.25, 500);
                telemetry.addLine("Right");
                telemetry.update();
                sleep(500);
            } else if (locationOfTSE == "middle") {
                robot.crane.setTargetPosition(-550);
                robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.crane.setPower(1);
                telemetry.addData("Ecoder value", robot.crane.getCurrentPosition());
                telemetry.update();
                robot.move(.25, 500);
                telemetry.addLine("Middle");
                telemetry.update();
                sleep(500);
            } else if (locationOfTSE == "left") {
                robot.crane.setTargetPosition(-300);
                robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.crane.setPower(1);
                robot.move(.25, 500);
                telemetry.addLine("Left");
                telemetry.update();
                sleep(300);
            } else if (locationOfTSE == "not Found") {

                sleep(400);
                telemetry.clearAll();
                telemetry.addData("one", barcode1);
                telemetry.addData("two", barcode2);
                telemetry.update();

                //for distance
                if (barcode1 <= 45 && barcode1 >= 30) {
                    telemetry.addLine("Right");
                    telemetry.update();
                    sleep(500);
                } else if (barcode2 <= 50 && barcode2 >= 30) {
                    robot.crane.setTargetPosition(-550);
                    robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.crane.setPower(.5);
                    telemetry.addLine("Middle");
                    telemetry.update();
                    sleep(400);
                } else {
                    robot.crane.setTargetPosition(-300);
                    robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.crane.setPower(.5);
                    telemetry.addLine("Left");
                    telemetry.update();
                    sleep(300);
                }
            }
            robot.move(.1, 400);

            //strafeLeft(1, 950);

            robot.gyroTurning(90, 1500L);

            robot.move(.8, 500);

            robot.gyroTurning(0, 1500L);

            sleep(500);

            //forward
            robot.move(.4, 600);


            //basic sleeping to make sure we are turning the motors as soon as the robot stops
            sleep(500);

            //open arm
            robot.arm.setPosition(1);
            sleep(500);


            //back up to wall
            robot.move(-.5, 800);

            sleep(500);

            robot.move(.3, 1000);

            //turn to warehouse
            robot.gyroTurning(-90, 1500L);

            sleep(500);

            //move to warehouse
            robot.move(1, 1300);

            robot.gyroTurning(-90, 1500L);

            robot.strafeLeft(0.75, 500);

            robot.gyroTurning(-90, 1500L);

            robot.move(0.5, 1000);
        }
        webcam.stopStreaming();
    }
}