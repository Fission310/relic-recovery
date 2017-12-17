package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * CraigLauncherTeleop is the primary TeleOp OpMode for Velocity Vortex. All driver-controlled
 * actions should be defined in this class.
 */
@TeleOp(name="Teleop: CraigLauncher", group="Teleop")
public class CraigLauncherTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private boolean slowDrive = false;
    private boolean slowDriveDebounce = false;

    /**
     * Runs the teleop code.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        DcMotor slides1 = hardwareMap.dcMotor.get("slides1");
        DcMotor slides2 = hardwareMap.dcMotor.get("slides2");

        Servo capBallRight = hardwareMap.servo.get("capBallRight");
        Servo capBallLeft = hardwareMap.servo.get("capBallLeft");

        DcMotor shooter1 = hardwareMap.dcMotor.get("shooter1");

        Servo rotate = hardwareMap.servo.get("rotate");

        /*
        Servo hopper = hardwareMap.servo.get("hopper");
        */


        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        rotate.setPosition(0.1);
        capBallLeft.setPosition(1);
        capBallRight.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && runtime.seconds() < 120.0) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            /*
            Joystick Map
            DRIVER
            Joysticks - drivetrain
            Linear slides down (hold)- left trigger
            Linear slides up (hold) - right trigger
            Cap ball servos (hold) - left bumper
            Cap ball mechanism release - B
            Shooter (hold) - A
            Slow drive - X
            OPERATOR
            same except no driving capability
             */

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            if (slowDrive) {
                double slowSpeed = 0.5;
                leftFront.setPower(-gamepad1.left_stick_y * slowSpeed);
                leftBack.setPower(-gamepad1.left_stick_y * slowSpeed);
                rightFront.setPower(-gamepad1.right_stick_y * slowSpeed);
                rightBack.setPower(-gamepad1.right_stick_y * slowSpeed);
            } else {
                leftFront.setPower(-gamepad1.left_stick_y);
                leftBack.setPower(-gamepad1.left_stick_y);
                rightFront.setPower(-gamepad1.right_stick_y);
                rightBack.setPower(-gamepad1.right_stick_y);
            }


            // linear slides down
            if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                slides1.setPower(-1);
                slides2.setPower(1);
            } else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                slides1.setPower(1);
                slides2.setPower(-1);
            } else {
                slides1.setPower(0);
                slides2.setPower(0);
            }

            // cap ball servo
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                capBallLeft.setPosition(0);
                capBallRight.setPosition(1);
            } else {
                capBallLeft.setPosition(1);
                capBallRight.setPosition(0);
            }

            /*
            // hopper servo
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                hopper.setPosition(1);
            } else {
                hopper.setPosition(0.5);
            }
            */

            // cap ball rotate
            if (gamepad1.b || gamepad2.b) {
                rotate.setPosition(0.7);
            }

            // shooter
            if (gamepad1.a || gamepad2.a) {
                shooter1.setPower(1);
            } else {
                shooter1.setPower(0);
            }

            // slow drive
            if (gamepad1.x || gamepad2.x) {
                slowDriveDebounce = true;
            } else {
                if (slowDriveDebounce) {
                    slowDrive = !slowDrive;
                    slowDriveDebounce = false;
                }
            }

        }
    }
}