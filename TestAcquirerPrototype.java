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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name = "Concept: Scan Servo", group = "Concept")
public class AcquirerPrototype extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   10;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo leftServo;
    Servo rightServo;


    @Override
    public void runOpMode() {

        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        // Wait for the start button
        telemetry.addData(">", "Press Start." );
        telemetry.update();
        waitForStart();

        leftServo.setPosition(0);
        rightServo.setPosition(1);

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            if (abs(gamepad1.left_trigger) > 0.1) {
                // Keep stepping up until we hit the max value.
                leftServo.setPosition(leftServo.getPosition() + INCREMENT);
                rightServo.setPosition(rightServo.getPosition() - INCREMENT);
                if (leftServo.getPosition() >= MAX_POS ) {
                    leftServo.setPosition(MAX_POS);
                } else if (rightServo.getPosition() <= MIN_POS) {
                    rightServo.setPosition(MIN_POS);
                }
            }
            else if (abs(gamepad1.right_trigger) > 0.1) {
                // Keep stepping down until we hit the min value.
                leftServo.setPosition(leftServo.getPosition() - INCREMENT);
                rightServo.setPosition(rightServo.getPosition() + INCREMENT);
                if (rightServo.getPosition() >= MAX_POS ) {
                    rightServo.setPosition(MAX_POS);
                } else if (leftServo.getPosition() <= MIN_POS) {
                    leftServo.setPosition(MIN_POS);
                }
            }

            // Display the current value
            telemetry.addData("Left servo Position", "%5.2f", leftServo.getPosition());
            telemetry.addData("Right servo Position", "%5.2f", rightServo.getPosition());
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
