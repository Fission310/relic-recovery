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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Teleop: Main", group = "Teleop")
public class TeleopMain extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    HardwareMain robot = new HardwareMain();

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DcMotor slides;

    Servo bottomLeft;
    Servo bottomRight;
    Servo topLeft;
    Servo topRight;

    //BNO055IMU imu;

    //ColorSensor sensorColor;
    Servo arm;

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // State used for updating opMode.telemetry
    Orientation angles;
    Acceleration gravity;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeverRest 40
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;
    static final double     PCONSTANT               = 0.1;

    static final double BLUE = 115;
    static final double RED = 85;
    final double SCALE_FACTOR = 255;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Define and Initialize Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomLeft = hardwareMap.servo.get("bottomLeft");
        bottomRight = hardwareMap.servo.get("bottomRight");
        topLeft = hardwareMap.servo.get("topLeft");
        topRight = hardwareMap.servo.get("topRight");

        slides = hardwareMap.dcMotor.get("slides");
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

        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        */

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);

        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        arm = hardwareMap.servo.get("arm");
        //sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor");

        arm.setPosition(1);
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

        leftFront.setPower(gamepad1.left_stick_y);
        leftBack.setPower(gamepad1.left_stick_y);
        rightBack.setPower(gamepad1.right_stick_y);
        rightFront.setPower(gamepad1.right_stick_y);

        if (abs(gamepad1.left_trigger) > 0.1 || abs(gamepad2.left_trigger) > 0.1) {
            // Keep stepping up until we hit the max value.
            topLeft.setPosition(topLeft.getPosition() + INCREMENT);
            topRight.setPosition(topRight.getPosition() - INCREMENT);

            if (topLeft.getPosition() >= MAX_POS ) {
                topLeft.setPosition(MAX_POS);
            } else if (topRight.getPosition() <= MIN_POS) {
                topRight.setPosition(MIN_POS);
            }

        }
        else if (abs(gamepad1.right_trigger) > 0.1 || abs(gamepad2.right_trigger) > 0.1) {
            // Keep stepping down until we hit the min value.
            topLeft.setPosition(topLeft.getPosition() - INCREMENT);
            topRight.setPosition(topRight.getPosition() + INCREMENT);

            if (topRight.getPosition() >= MAX_POS ) {
                topRight.setPosition(MAX_POS);
            } else if (topLeft.getPosition() <= MIN_POS) {
                topLeft.setPosition(MIN_POS);
            }

        }

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            // Keep stepping up until we hit the max value.
            bottomLeft.setPosition(bottomLeft.getPosition() + INCREMENT);
            bottomRight.setPosition(bottomRight.getPosition() - INCREMENT);

            if (bottomLeft.getPosition() >= MAX_POS) {
                bottomLeft.setPosition(MAX_POS);
            } else if (bottomRight.getPosition() <= MIN_POS) {
                bottomRight.setPosition(MIN_POS);
            }

        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
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

        if (gamepad1.x || gamepad2.x) {
            arm.setPosition(0.1);
        } else if (gamepad1.y || gamepad2.y) {
            arm.setPosition(1);
        }
    }
}
