package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

import static java.lang.Math.abs;

/**
 * TeleopMain is the primary TeleOp OpMode for Relic Recovery. All driver-controlled actions should
 * be defined in this class.
 */
@TeleOp(name = "Teleop: Main", group = "Teleop")
public class TeleopMain extends OpMode {

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMain robot = new HardwareMain();

    /**
     * Runs once when the OpMode is first enabled. The robot's hardware map is initialized.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
    }


    /**
     * Runs continuously while OpMode is waiting to start.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init_loop()
     */
    @Override
    public void init_loop() { }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * Runs continuously while the OpMode is active. Defines the driver-controlled actions
     * according to gamepad input.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        // Adds runtime data to telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        // Drives the robot based on driver joystick input

        robot.drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Clamps acquirer servos if triggers are pressed
        /*
        if (abs(gamepad1.left_trigger) > 0.1 || abs(gamepad2.left_trigger) > 0.1) {
            // Steps up the servo value until the max position is reached.
            robot.acquirer.clampIncrement(true);
        } else if (abs(gamepad1.right_trigger) > 0.1 || abs(gamepad2.right_trigger) > 0.1) {
            // Steps down the servo value until the min position is reached.
            robot.acquirer.clampIncrement(false);
        }
        */

        // Moves slides if dpad up/down is pressed
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.relic.setSlidesPower(1);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.relic.setSlidesPower(-1);
        } else {
            robot.relic.setSlidesPower(0);
        }

        // Toggles arm from upright and down positions if start or back is pressed
        if (gamepad1.start || gamepad2.start) {
            robot.arm.getArm().setPosition(0.1);
        } else if (gamepad1.back || gamepad2.back) {
            robot.arm.getArm().setPosition(1);
        }

        // Toggles turn servo from up or down positions if a or b is pressed
        if (gamepad1.a || gamepad2.a) {
            robot.relic.turnDown();
        } else if (gamepad1.b || gamepad2.b) {
            robot.relic.turnUp();
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.relic.turnInside();
        }

        // Toggles clamp servo from up or down positions if x or y is pressed
        if (gamepad1.x || gamepad2.x) {
            robot.relic.clamp();
        } else if (gamepad1.y || gamepad2.y) {
            robot.relic.unclamp();
        }
    }
}
