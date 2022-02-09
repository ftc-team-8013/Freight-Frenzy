/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor: MR range sensor", group = "Sensor")
public class distance_test extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensorM;
    ModernRoboticsI2cRangeSensor rangeSensorR;

    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensorM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceM");
        rangeSensorR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceR");
        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("raw ultrasonic", rangeSensorM.rawUltrasonic());
//            telemetry.addData("raw optical", rangeSensorM.rawOptical());
//            telemetry.addData("cm optical", "%.2f cm", rangeSensorM.cmOptical());
            telemetry.addData("cmm", "%.2f", rangeSensorM.getDistance(DistanceUnit.CM));
            telemetry.update();

            telemetry.addData("cmr", "%.2f", rangeSensorR.getDistance(DistanceUnit.CM));

            if (rangeSensorM.getDistance(DistanceUnit.CM) >= 47 && rangeSensorM.getDistance(DistanceUnit.CM) <= 54) {
                telemetry.addLine("Middle");
            }else if (rangeSensorR.getDistance(DistanceUnit.CM) >= 47 && rangeSensorR.getDistance(DistanceUnit.CM) <= 60){
                telemetry.addLine("Right");
            }else {
                telemetry.addLine("Left");
            }
        }
    }
}