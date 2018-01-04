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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.hardware.tankdrive.Drivetrain;


/**
 * TODO: This is a work in progress.
 * AutonVuMark is a class containing the following autonomous routine for the BLUE alliance:
 * <ol>
 *   <li>Detect VuMark and determine target cryptobox column</li>
 *   <li>Score glyph into determined column</li>
 * </ol>
 */
@Autonomous(name="Auton: VuMark", group ="Auton")
public class AutonVuMark extends LinearOpMode {

    /* Robot hardware */
    private HardwareMain robot = new HardwareMain(this);

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {

        // To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Do not activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia license key
        parameters.vuforiaLicenseKey = "ATHlAS7/////AAAAGSrlghNuCkIdu0Y/Eqnxz9oejoRzibKYqWJYEJik+9ImrFuJaDs2/WAm5ovuC4iV/m4DHM3WWgAl9pI5MQULOsKslna/+bYWzcbzpzak4NMtWuGLnnJYCeH8vP2x8fC8R0I+Odvd4vhnJdSa3P6C87oTqtVSX0sZcVOvALmUpCJcSFHAqshW0F7XziW89qM4tBDQoKgNCkbFNmKeRnKa4j4Vfyk0RSNXc/79shIk8Pu4j8krsBComGYTx4FKsClnfgZYOp51uhMg/yoEHfpy0XMrCOBZUYIyTVvOsCtC9GzLAOLoxEnunRRjagCKni32kkrH07slhuiCqpNBJQ02y8qZFChTjt5i+ZZwnzaWCFSf";

        // Use back camera and initialize
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // Load the data set containing the VuMarks for Relic Recovery
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Output for Driver Station
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        // Run while OpMode is active
        while (opModeIsActive()) {

            /*
             * See if any of the instances of the relic template are currently visible.
             * vuMark can have the following values: UNKNOWN, LEFT, CENTER, and RIGHT.
             * When a VuMark is visible, something other than UNKNOWN will be returned.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            while (vuMark == RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "not visible");
                telemetry.update();

                // Drive forward one inch
                robot.drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 1, 1, 0.75);

                // Look for relic template
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }

            // Found an instance of the template
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();

            // Find the target column based on VuMark
            int targetCol;
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                targetCol = 0;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                targetCol = 1;
            } else {
                targetCol = 2;
            }

            // Run glyph scoring action
            robot.scoreGlyph(targetCol);

        }
    }

}
