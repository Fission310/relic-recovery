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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * ConceptVuforiaFollowTarget is a class containing an autonomous program that makes a robot follow
 * an image target. A phone mounted on the robot uses Vuforia to detect the image, and the robot's
 * distance and angle to the target is calculated from the raw camera vision data. The robot moves
 * towards the target based on these calculations.
 */
@Autonomous(name="Concept: Vuforia Target Follow", group ="Concept")
public class ConceptVuforiaFollowTarget extends LinearOpMode {

    /* Constants */
    private float mmPerInch        = 25.4f;
    private float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    private float mmFTCFieldWidth  = (12 * 12) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

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

        // Load the data set containing the targets for tracking
        VuforiaTrackables stonesAndChips = vuforia.loadTrackablesFromAsset("StonesAndChips");
        VuforiaTrackable redTarget = stonesAndChips.get(0);
        redTarget.setName("RedTarget");  // Stones

        //VuforiaTrackable blueTarget  = stonesAndChips.get(1);
        //blueTarget.setName("BlueTarget");  // Chips

        /* For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.add(redTarget);

        // Initialize red target at a place on the "field"; this does not matter for this program
        OpenGLMatrix redTargetLocationOnField = createMatrix(0, mmFTCFieldWidth/2, 0, 0, 90, 0);
        redTarget.setLocation(redTargetLocationOnField);
        float[] targetCoords = redTargetLocationOnField.getTranslation().getData();
        float targetX = targetCoords[0];
        float targetY = targetCoords[1];

        // Initialize phone location on robot as upright, halfway across the length of the robot
        OpenGLMatrix phoneLocationOnRobot = createMatrix(0, mmBotWidth/2, 0, 0, 90, 0);

        // Let the trackable listeners know where the phone is.
        ((VuforiaTrackableDefaultListener)redTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        // Initialize robot hardware
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for start of OpMode
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        // Start tracking the data sets
        stonesAndChips.activate();

        // Variables to store target location from robot
        float robotX = 0;
        float robotY = 0;
        float robotAngle = 0;

        // Run while OpMode is active
        while (opModeIsActive()) {

            // Variable to store whether the target is visible
            boolean visible = false;

            for (VuforiaTrackable trackable : allTrackables) {

                // Check if target is visible and output data to user
                visible = ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible();
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");

                // Get updated robot location and store in variables
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {

                    float[] coordinates = robotLocationTransform.getTranslation().getData();

                    robotX = coordinates[0];
                    robotY = coordinates[1];
                    robotAngle = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
                }
            }

            // Output updated robot location to user
            telemetry.addData("X1: ", robotX);
            telemetry.addData("X2: ", targetX);
            telemetry.addData("Y1: ", robotY);
            telemetry.addData("Y2: ", targetY);
            telemetry.addData("Angle: ", robotAngle);
            telemetry.addData("Distance: ", distance(robotX, targetX, robotY, targetY));
            telemetry.update();

            /*
             * If distance is sufficiently great and the target is visible, move the robot towards
             * the target. Otherwise, don't move.
             */
            if (distance(robotX, targetX, robotY, targetY) > 500 && visible) {
                if (robotAngle > 3) {
                    leftFront.setPower(0.3);
                    leftBack.setPower(0.3);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                } else {
                    rightFront.setPower(0.3);
                    rightBack.setPower(0.3);
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                }
            } else if (distance(robotX, targetX, robotY, targetY) < 500) {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            } else if (!visible) {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            } else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }

        }
    }

    /**
     * Calculate and return the location matrix of a field object based on the phone's location
     * on the robot.
     *
     * @param x     x-coordinate of translation
     * @param y     y-coordinate of translation
     * @param z     z-coordinate of translation
     * @param u     degrees of rotation in x
     * @param v     degrees of rotation in y
     * @param w     degrees of rotation in z
     * @return      recalculate location matrix based on phone location on robot
     */
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w
                ));
    }

    /**
     * Uses the distance formula to calculate the distance between two points in an xy-coordinate
     * system.
     *
     * @param x1    first x coordinate
     * @param x2    second x coordinate
     * @param y1    first y coordinate
     * @param y2    second y coordinate
     * @return      calculated distance between (x1, y1) and (x2, y2)
     */
    private double distance(float x1, float x2, float y1, float y2) {
        return Math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }
}
