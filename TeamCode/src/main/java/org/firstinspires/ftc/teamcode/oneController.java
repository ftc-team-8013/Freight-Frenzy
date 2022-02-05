/*
// This is a simple program which we will use to test values, driving styles, and other things. This is not our final version of the TeleOp but this will definitely contribute to it.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled

public class oneController extends LinearOpMode {

    //Motor initialization
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor carousel;
    private DcMotor crane;
    private Servo arm;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        carousel = hardwareMap.get(DcMotor.class, "carousel");
        crane = hardwareMap.get(DcMotor.class, "crane");
        arm = hardwareMap.get(Servo.class, "arm");

        //setting motor direction since some motors were backwards
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            //defining driving variables.
            double turn;
            double throttle;
            boolean strafeLeft;
            boolean strafeRight;

            boolean craneUp;
            boolean armToggle;
            double carouselPower;
            boolean craneDown;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeLeft = gamepad1.left_bumper;
            strafeRight = gamepad1.right_bumper;

            craneUp = gamepad1.dpad_up;
            craneDown = gamepad1.dpad_down;
            armToggle = gamepad1.x;
            carouselPower = gamepad1.left_trigger;

            //making motors run.
            //strafing
            if (strafeLeft) {
                frontLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backLeft.setPower(0.75);
                backRight.setPower(-0.75);
            } else if (strafeRight) {
                frontLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backLeft.setPower(0.75);
                backRight.setPower(-0.75);
            }
            //forward and backward movement
            frontLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backLeft.setPower(throttle);
            backRight.setPower(throttle);

            //turning
            frontLeft.setPower(-turn);
            frontRight.setPower(turn);
            backLeft.setPower(-turn);
            backRight.setPower(turn);

            if (craneUp){
                crane.setPower(0.5);
            }else if (craneDown){
                crane.setPower(-0.5);
            }else{
                crane.setPower(0);
            }

            if (armToggle){
                arm.setPosition(0);
            }else{
                arm.setPosition(1);
            }

            carousel.setPower(carouselPower);
            System.out.println("servo position "+ arm.getPosition());
            telemetry.update();
        }
    }
}
