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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Teleop: Main", group = "Teleop")
public class TeleopMain extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    HardwareMain robot = new HardwareMain();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);        
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() { }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        // TODO: add if button held while start, clamp servos together
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        robot.leftFront.setPower(gamepad1.left_stick_y);
        robot.leftBack.setPower(gamepad1.left_stick_y);
        robot.rightBack.setPower(gamepad1.right_stick_y);
        robot.rightFront.setPower(gamepad1.right_stick_y);

        if (abs(gamepad1.left_trigger) > 0.1 || abs(gamepad2.left_trigger) > 0.1) {
            // Keep stepping up until we hit the max value.
            robot.topLeft.setPosition(robot.topLeft.getPosition() + HardwareMain.INCREMENT);
            robot.topRight.setPosition(robot.topRight.getPosition() - HardwareMain.INCREMENT);

            if (robot.topLeft.getPosition() >= HardwareMain.MAX_POS ) {
                robot.topLeft.setPosition(HardwareMain.MAX_POS);
            } else if (robot.topRight.getPosition() <= HardwareMain.MIN_POS) {
                robot.topRight.setPosition(HardwareMain.MIN_POS);
            }

        }
        else if (abs(gamepad1.right_trigger) > 0.1 || abs(gamepad2.right_trigger) > 0.1) {
            // Keep stepping down until we hit the min value.
            robot.topLeft.setPosition(robot.topLeft.getPosition() - HardwareMain.INCREMENT);
            robot.topRight.setPosition(robot.topRight.getPosition() + HardwareMain.INCREMENT);

            if (robot.topRight.getPosition() >= HardwareMain.MAX_POS ) {
                robot.topRight.setPosition(HardwareMain.MAX_POS);
            } else if (robot.topLeft.getPosition() <= HardwareMain.MIN_POS) {
                robot.topLeft.setPosition(HardwareMain.MIN_POS);
            }

        }

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            // Keep stepping up until we hit the max value.
            robot.bottomLeft.setPosition(robot.bottomLeft.getPosition() + HardwareMain.INCREMENT);
            robot.bottomRight.setPosition(robot.bottomRight.getPosition() - HardwareMain.INCREMENT);

            if (robot.bottomLeft.getPosition() >= HardwareMain.MAX_POS) {
                robot.bottomLeft.setPosition(HardwareMain.MAX_POS);
            } else if (robot.bottomRight.getPosition() <= HardwareMain.MIN_POS) {
                robot.bottomRight.setPosition(HardwareMain.MIN_POS);
            }

        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            // Keep stepping down until we hit the min value.
            robot.bottomLeft.setPosition(robot.bottomLeft.getPosition() - HardwareMain.INCREMENT);
            robot.bottomRight.setPosition(robot.bottomRight.getPosition() + HardwareMain.INCREMENT);

            if (robot.bottomRight.getPosition() >= HardwareMain.MAX_POS) {
                robot.bottomRight.setPosition(HardwareMain.MAX_POS);
            } else if (robot.bottomLeft.getPosition() <= HardwareMain.MIN_POS) {
                robot.bottomLeft.setPosition(HardwareMain.MIN_POS);
            }

        }

        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.slides.setPower(-1);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.slides.setPower(1);
        } else {
            robot.slides.setPower(0);
        }

        if (gamepad1.x || gamepad2.x) {
            robot.arm.setPosition(0);
        } else if (gamepad1.y || gamepad2.y) {
            robot.arm.setPosition(1);
        }
    }
}
