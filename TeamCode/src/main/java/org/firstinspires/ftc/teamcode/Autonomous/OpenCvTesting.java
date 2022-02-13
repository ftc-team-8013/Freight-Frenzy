package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Skystone Detecotor", group="Auto")
public class OpenCvTesting extends LinearOpMode {
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = hardwareMap.get(OpenCvWebcam.class,  "Webcam 1");
        OpenCV detector = new OpenCV(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(
                () -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );

        waitForStart();
        switch (detector.getLocation()) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }
        webcam.stopStreaming();
    }
}