
//TODO: Test Code

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@Autonomous

public class blueRightCarousel extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    //Init motors.
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor carousel;
    DcMotor crane;
    Servo arm;
    TouchSensor Touch;

    public void runOpMode() {
        initDriveMotors();
        initMiscMotors();
        initGyro();

        waitForStart();
        telemetry.addLine("Y'all ready for this?");
        telemetry.update();

        if (opModeIsActive()) {
            //closing the arm and waiting so we know the block is in possession
            arm.setPosition(0);
            sleep(400);

            //move forwards a few inches
            move(0.25, 500);

            //turning 90 degrees counterclockwise
            gyroTurning(90);

            //frontLeft.setPower(-0.5);
            //backLeft.setPower(-0.5);
            //frontRight.setPower(0.5);
           //backRight.setPower(0.5);
            //sleep(710);
            //stopMotors();

            //reverse back into carousel
            move(-0.4, 600);
            sleep(150);

            //turns on the carousel motor to get the duck onto the floor
            //spin while moving forward
            carouselMotor(0.9, 1700);

            //turning on the crane motor making the crane go up and avoid the terrain
            crane.setPower(-0.5);
            sleep(500);

            //move toward warehouse
            move(0.5, 1400);

            //turning to shipping hub
            gyroTurning(0);
            sleep(500);

            //move to delivery
            move(0.25, 1330);
            sleep(550);

            //open claw
            arm.setPosition(1);
            sleep(550);

            //move back from shipping hub
            move(-0.5, 300);


            //turning off crane motor when in the down position
            while (Touch.isPressed() != true) {
                crane.setPower(1);
            }
            crane.setPower(0);

            // turn 45
            gyroTurning(50);
            sleep(300);
            //tweak

            move(.3, 770);
            sleep(150);

            arm.setPosition(0);
            sleep(500);

            crane.setPower(1);
            sleep(500);

            gyroTurning(-20);
            sleep(300);

            move(.25, 400);

            arm.setPosition(1);
            sleep(300);

            //gyroTurning(90);

            frontLeft.setPower(-0.6);
            backLeft.setPower(-0.6);
            frontRight.setPower(0.6);
            backRight.setPower(0.6);
            sleep(650);
            stopMotors();

            move(1, 1500);

            // small reverse to stop.
            move (-0.5, 100);
        }
    }

    //Init methods
    public void initDriveMotors() {
        //Setting variables in code to a motor in the configuration.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        Touch = hardwareMap.get(TouchSensor.class, "Touch" );

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
    }

    //Movement methods
    public boolean gyroTurning(double targetAngle) {
        boolean foundAngle = false;
        while (opModeIsActive()) {
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
                    frontLeft.setPower(0.2);
                    frontRight.setPower(-0.2);
                    backLeft.setPower(0.2);
                    backRight.setPower(-0.2);
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
                    frontLeft.setPower(-0.2);
                    frontRight.setPower(0.2);
                    backLeft.setPower(-0.2);
                    backRight.setPower(0.2);
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

    //Other method
    public void carouselMotor(double power, int time){
        carousel.setPower(power);
        sleep(time);
        carousel.setPower(0);
    }
}
