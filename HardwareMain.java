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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
 */
public class HardwareMain {

    /* Public OpMode members. */
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DcMotor slides;

    Servo bottomLeft;
    Servo bottomRight;
    Servo topLeft;
    Servo topRight;

    BNO055IMU imu;

    ColorSensor sensorColor;
    Servo arm;

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS = 0.7;     // Maximum rotational position
    static final double MIN_POS = 0.3;     // Minimum rotational position

    // State used for updating opMode.telemetry
    Orientation angles;
    Acceleration gravity;

    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // NeverRest 40
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;
    static final double     PCONSTANT               = 0.1;

    static final double BLUE = 120;
    static final double RED = 120;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareMain(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomLeft = hwMap.servo.get("bottomLeft");
        bottomRight = hwMap.servo.get("bottomRight");
        topLeft = hwMap.servo.get("topLeft");
        topRight = hwMap.servo.get("topRight");

        slides = hwMap.dcMotor.get("slides");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        slides.setPower(0);

        topLeft.setPosition(MAX_POS);
        topRight.setPosition(MIN_POS);
        bottomLeft.setPosition(MAX_POS);
        bottomRight.setPosition(MIN_POS);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        arm = hwMap.servo.get("arm");
        sensorColor = hwMap.get(ColorSensor.class, "color_sensor");

        arm.setPosition(1);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(LinearOpMode opMode,
                             double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        double acc = 0;
        double t = 0;

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        leftFront.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                leftFront.isBusy() && rightFront.isBusy()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double gyroAngle = angles.firstAngle;

            double p = (gyroAngle - currentAngle) * PCONSTANT;

            leftFront.setPower(Math.abs(speed) + p);
            rightFront.setPower(Math.abs(speed) - p);
                /*
                if (leftInches < 0) {
                    leftBack.setPower(-Math.abs(speed));
                } else {
                    leftBack.setPower(Math.abs(speed));
                }
                if (rightInches < 0) {
                    rightBack.setPower(-Math.abs(speed));
                } else {
                    rightBack.setPower(Math.abs(speed));
                }*/

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test

            // Display it for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition());
            opMode.telemetry.addData("Heading: ", "%f", gyroAngle);
            opMode.telemetry.addData("AccX: ", "%f", imu.getAcceleration().xAccel);
            opMode.telemetry.addData("AccY: ", "%f", imu.getAcceleration().yAccel);
            opMode.telemetry.update();

            t = runtime.seconds();
            acc = Math.sqrt(imu.getAcceleration().xAccel * imu.getAcceleration().xAccel + imu.getAcceleration().yAccel * imu.getAcceleration().yAccel);
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);

        opMode.telemetry.addData("Distance: ", "%f", 0.5 * acc * t * t);
        opMode.telemetry.addData("Angle: ", "%f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        opMode.telemetry.update();

        // sleep(1000);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(0);
        rightFront.setPower(0);
    }


    public void turn(LinearOpMode opMode, double speed, double angle, double timeoutS) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angleToTurn = angle - angles.firstAngle;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opMode.opModeIsActive() && (Math.abs(angleToTurn) > 0.5 || runtime.seconds() < timeoutS)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleToTurn = angle - angles.firstAngle;

            leftFront.setPower(-Math.signum(angleToTurn) * speed);
            rightFront.setPower(Math.signum(angleToTurn) * speed);
            leftBack.setPower(-Math.signum(angleToTurn) * speed);
            rightBack.setPower(Math.signum(angleToTurn) * speed);

            opMode.telemetry.addData("Heading: ", "%f", angleToTurn);
            opMode.telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        opMode.telemetry.addData("Angle: ", "%f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        opMode.telemetry.update();
    }


    /*
    Assumes the color sensor faces the FRONT of the bot.
    If reversed, negate drive inches.
    Returns the number of inches moved, with respect to the original position.
     */
    public int jewel(LinearOpMode opMode, boolean isAllianceRed) {
        if (opMode.opModeIsActive()) {
            arm.setPosition(0.1);
            opMode.sleep(1000);
            opMode.telemetry.addData("Alpha", sensorColor.alpha());
            opMode.telemetry.addData("Red  ", sensorColor.red());
            opMode.telemetry.addData("Green", sensorColor.green());
            opMode.telemetry.addData("Blue ", sensorColor.blue());
            opMode.sleep(1500);
        }
        if (opMode.opModeIsActive()) {
            int inchesToDrive = 3;
            if (sensorColor.blue() > BLUE) {
                // blue detected
                if (isAllianceRed) {
                    // drive forward
                    encoderDrive(opMode, DRIVE_SPEED, inchesToDrive, inchesToDrive, 2);
                    return inchesToDrive;
                } else {
                    encoderDrive(opMode, DRIVE_SPEED, -inchesToDrive, -inchesToDrive, 2);
                    return -inchesToDrive;
                }
            } else if (sensorColor.red() > RED) {
                if (isAllianceRed) {
                    encoderDrive(opMode, DRIVE_SPEED, -inchesToDrive, -inchesToDrive, 2);
                    return inchesToDrive;
                } else {
                    encoderDrive(opMode, DRIVE_SPEED, inchesToDrive, inchesToDrive, 2);
                    return -inchesToDrive;
                }
            }
        }

        return 0;
    }


    public void clamp(LinearOpMode opMode) {
        if (opMode.opModeIsActive()) {
            topLeft.setPosition(MAX_POS / 3);
            topRight.setPosition((1 - MIN_POS) * 2 / 3);
            bottomLeft.setPosition(MAX_POS / 3);
            bottomRight.setPosition((1 - MIN_POS) * 2 / 3);
        }
    }


    public void release(LinearOpMode opMode) {
        if (opMode.opModeIsActive()) {
            topLeft.setPosition(MAX_POS);
            topRight.setPosition(MIN_POS);
            bottomLeft.setPosition(MAX_POS);
            bottomRight.setPosition(MIN_POS);
        }
    }

 }

