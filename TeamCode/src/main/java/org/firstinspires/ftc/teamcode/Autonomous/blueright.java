package org.firstinspires.ftc.teamcode.Autonomous;

//IMPORTS
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.robotClass;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="blueright close parking", group="Auto")
public class blueright extends LinearOpMode {
    OpenCvWebcam webcam;

    //defining varibles
    robotClass robot = new robotClass();

    @Override
    public void runOpMode() throws InterruptedException {
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
            //close claw
            robot.arm.setPosition(0);
            sleep(1500);
            //crane up out of the way
            robot.crane.setTargetPosition(-900);
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

                    barcode1 = robot.rangeSensorM.cmUltrasonic();
                    sleep(400);

                    robot.gyroTurning(12, 1000);
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
                robot.crane.setPower(.5);
                robot.move(.25, 500);
                telemetry.addLine("Middle");
                telemetry.update();
                sleep(500);
            } else if (locationOfTSE == "left") {
                robot.crane.setTargetPosition(-300);
                robot.crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.crane.setPower(.5);
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
            //turning 90 degrees counterclockwise to carousel
            robot.gyroTurning(90, 1500);

            //reverse back into carousel
            robot.move(-0.3, 1300);

            //turns on the carousel motor to get the duck onto the floor
            robot.carouselMotor(1, 2000);

            //moving to warehouse
            robot.move(0.5, 1660);

            //turning to shipping hub
            robot.gyroTurning(0, 1500);
            sleep(500);

            //move to delivery
            robot.move(0.25, 1000);
            sleep(250);

            //open claw
            robot.arm.setPosition(1);
            sleep(750);

            //move back from shipping hub
            robot.move(-0.5, 780);
            sleep(200);

            robot.move(.3, 300);

            // turn 90
            robot.gyroTurning(-90, 1500);

            //move to park
            robot.frontLeft.setPower(-1);
            robot.frontRight.setPower(-.85);
            robot.backLeft.setPower(-1);
            robot.backRight.setPower(-.85);
            sleep(1100);
            robot.stopMotors();

            robot.gyroTurning(0, 1500);

            robot.move(-.5, 500);
        }
        webcam.stopStreaming();
    }
}