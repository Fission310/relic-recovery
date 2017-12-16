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

package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Test: Acquirer Prototype", group = "Test")
public class TestAcquirerPrototype extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   10;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo topLeft;
    Servo topRight;
    Servo bottomLeft;
    Servo bottomRight;

    DcMotor slides;

    @Override
    public void runOpMode() {

        topLeft = hardwareMap.get(Servo.class, "topLeft");
        topRight = hardwareMap.get(Servo.class, "topRight");
        bottomLeft = hardwareMap.get(Servo.class, "bottomLeft");
        bottomRight = hardwareMap.get(Servo.class, "bottomRight");

        slides = hardwareMap.get(DcMotor.class, "slides");

        // Wait for the start button
        telemetry.addData(">", "Press Start." );
        telemetry.update();
        waitForStart();

        topLeft.setPosition(1);
        topRight.setPosition(0);
        bottomLeft.setPosition(1);
        bottomRight.setPosition(0);

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            if (abs(gamepad1.left_trigger) > 0.1) {
                // Keep stepping up until we hit the max value.
                topLeft.setPosition(topLeft.getPosition() + INCREMENT);
                topRight.setPosition(topRight.getPosition() - INCREMENT);
                if (topLeft.getPosition() >= MAX_POS ) {
                    topLeft.setPosition(MAX_POS);
                } else if (topRight.getPosition() <= MIN_POS) {
                    topRight.setPosition(MIN_POS);
                }
            }
            else if (abs(gamepad1.right_trigger) > 0.1) {
                // Keep stepping down until we hit the min value.
                topLeft.setPosition(topLeft.getPosition() - INCREMENT);
                topRight.setPosition(topRight.getPosition() + INCREMENT);
                if (topRight.getPosition() >= MAX_POS ) {
                    topRight.setPosition(MAX_POS);
                } else if (topLeft.getPosition() <= MIN_POS) {
                    topLeft.setPosition(MIN_POS);
                }
            }

            if (gamepad1.left_bumper) {
                // Keep stepping up until we hit the max value.
                bottomLeft.setPosition(bottomLeft.getPosition() + INCREMENT);
                bottomRight.setPosition(bottomRight.getPosition() - INCREMENT);
                if (bottomLeft.getPosition() >= MAX_POS) {
                    bottomLeft.setPosition(MAX_POS);
                } else if (bottomRight.getPosition() <= MIN_POS) {
                    bottomRight.setPosition(MIN_POS);
                }
            } else if (gamepad1.right_bumper) {
                // Keep stepping down until we hit the min value.
                bottomLeft.setPosition(bottomLeft.getPosition() - INCREMENT);
                bottomRight.setPosition(bottomRight.getPosition() + INCREMENT);
                if (bottomRight.getPosition() >= MAX_POS) {
                    bottomRight.setPosition(MAX_POS);
                } else if (bottomLeft.getPosition() <= MIN_POS) {
                    bottomLeft.setPosition(MIN_POS);
                }
            }

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                slides.setPower(-1);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                slides.setPower(1);
            } else {
                slides.setPower(0);
            }

            // Display the current value
            telemetry.addData("Left servo Position", "%5.2f", topLeft.getPosition());
            telemetry.addData("Right servo Position", "%5.2f", topRight.getPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
