/*
// This is a simple program which we will use to test values, driving styles, and other things. This is not our final version of the TeleOp but this will definitely contribute to it.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class chassisTest extends LinearOpMode {

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
            double strafeValue;
            double throttle;

            double cranePower;
            boolean armIn = false;
            boolean armOut = false;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeValue = gamepad1.left_stick_x;

            cranePower = gamepad2.left_stick_y;
            armIn = gamepad2.dpad_right;
            armOut = gamepad2.dpad_left;

            //making motors run.
            //strafing
            if (strafeValue < -0.1) {
                frontLeft.setPower(-strafeValue);
                frontRight.setPower(strafeValue);
                backLeft.setPower(strafeValue);
                backRight.setPower(-strafeValue);
            } else if (strafeValue > 0.1) {
                frontLeft.setPower(-strafeValue);
                frontRight.setPower(strafeValue);
                backLeft.setPower(strafeValue);
                backRight.setPower(-strafeValue);
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

            if (cranePower > 0.2 || cranePower < -0.2){
                crane.setPower(cranePower);
            }else{
                crane.setPower(-0.1);
            }

            if (armIn) {
                arm.setPosition(0.55);
            } else if (armOut) {
                arm.setPosition(0.18);
            }

            

            telemetry.addData("Elevator Trim", cranePower);
            System.out.println("servo position "+ arm.getPosition());
            telemetry.update();
        }
    }
}
