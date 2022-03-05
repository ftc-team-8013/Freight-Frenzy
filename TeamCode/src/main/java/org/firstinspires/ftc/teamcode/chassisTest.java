/*
// This is a simple program which we will use to test values, driving styles, and other things. This is not our final version of the TeleOp but this will definitely contribute to it.
 */
package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class chassisTest extends LinearOpMode {
    functions robot = new functions();
    //Motor initialization

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("time", System.currentTimeMillis());

            //defining driving variables.
            double turn;
            double throttle;
            boolean strafeLeft;
            boolean strafeRight;

            double cranePower;
            boolean armToggle;
            double carouselPower;
            double redCarouselPower;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeLeft = gamepad1.left_bumper;
            strafeRight = gamepad1.right_bumper;

            cranePower = gamepad2.left_stick_y;
            armToggle = gamepad2.x;
            carouselPower = gamepad2.left_trigger;
            redCarouselPower = gamepad2.right_trigger;

            //making motors run.
            //strafing
            if (strafeLeft) {
                robot.frontLeft.setPower(1);
                robot.frontRight.setPower(-1);
                robot.backLeft.setPower(-1);
                robot.backRight.setPower(1);
            } else if (strafeRight) {
                robot.frontLeft.setPower(-1);
                robot.frontRight.setPower(1);
                robot.backLeft.setPower(1);
                robot.backRight.setPower(-1);
            }
            //forward and backward movement
            robot.frontLeft.setPower(-throttle);
            robot.frontRight.setPower(-throttle);
            robot.backLeft.setPower(-throttle);
            robot.backRight.setPower(-throttle);

            //turning
            robot.frontLeft.setPower(turn);
            robot.frontRight.setPower(-turn);
            robot.backLeft.setPower(turn);
            robot.backRight.setPower(-turn);

            robot.crane.setPower(cranePower);

            if (armToggle){
                robot.arm.setPosition(0);
            }else{
                robot.arm.setPosition(1);
            }
            robot.redCarousel.setPower(-redCarouselPower);
            robot.carousel.setPower(carouselPower);
            telemetry.addData("Elevator Trim", cranePower);
            System.out.println("servo position "+ robot.arm.getPosition());
            telemetry.update();
        }
    }
}
