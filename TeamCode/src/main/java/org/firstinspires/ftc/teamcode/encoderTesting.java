package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class encoderTesting extends LinearOpMode {
    private DcMotor crane;

    public void runOpMode(){
        crane = hardwareMap.get(DcMotor.class, "crane");

        waitForStart();

        while (opModeIsActive()){
            double throttle;
            throttle = gamepad1.left_stick_y;

            crane.setPower(throttle);
            telemetry.addData("encoder value", crane.getCurrentPosition());
            telemetry.update();
        }
    }
}
