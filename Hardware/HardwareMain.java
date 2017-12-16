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

package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** TODO: UPDATE
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 *
 * TODO AUTONS:
 *  - Jewel + Safe zone park for all configurations
 *    - Blue left/right stone back park
 *    - Blue left stone forward park
 *    - Red left/right stone forward park
 *    - Red right stone back park
 *  - Jewel + Blue cryptobox score left (random)
 *  - Jewel + Red cryptobox score left (random)
 */
public class HardwareMain {

    /* Public OpMode members. */
    public Drivetrain drivetrain = new Drivetrain();
    public Acquirer acquirer = new Acquirer();
    public Arm arm = new Arm();

    private LinearOpMode opMode;

    DistanceSensor sensorDistance;

    /* Constructor */
    public HardwareMain(){

    }
    public HardwareMain(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        acquirer.init(hwMap);
        arm.init(hwMap);

        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");
    }

    /*
    Assumes the color sensor faces the FRONT of the bot.
    If reversed, negate drive inches.
    Returns the number of inches moved, with respect to the original position.
     */
    public int jewel(boolean isAllianceRed) {
        if (opMode.opModeIsActive()) {
            arm.getArm().setPosition(0.1);
            float[] hsvValues = arm.getHSVValues();
            opMode.telemetry.addData("HSV: ", hsvValues);
            opMode.telemetry.update();
            opMode.sleep(1000);

            int inchesToDrive = 5;
            hsvValues = arm.getHSVValues();
            if (hsvValues[0] > Arm.BLUE) {
                // blue detected
                if (isAllianceRed) {
                    // drive forward
                    drivetrain.encoderDrive(opMode, Drivetrain.DRIVE_SPEED, -inchesToDrive, -inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    return inchesToDrive;
                } else {
                    drivetrain.encoderDrive(opMode, Drivetrain.DRIVE_SPEED, inchesToDrive, inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    return -inchesToDrive;
                }
            } else if (hsvValues[0] < Arm.RED) {
                if (isAllianceRed) {
                    drivetrain.encoderDrive(opMode, Drivetrain.DRIVE_SPEED, inchesToDrive, inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    return inchesToDrive;
                } else {
                    drivetrain.encoderDrive(opMode, Drivetrain.DRIVE_SPEED, -inchesToDrive, -inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    return -inchesToDrive;
                }
            }
        }

        arm.getArm().setPosition(0.1);
        return 0;
    }

    public void scoreGlyph(int targetCol) {
        int wallsDetected = -1;
        while (opMode.opModeIsActive() && wallsDetected < targetCol) {
            double boxDistance = sensorDistance.getDistance(DistanceUnit.CM);
            if (boxDistance < 10) {
                wallsDetected++;
            }
        }
        // score
    }

 }

