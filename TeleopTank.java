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
@TeleOp(name = "Teleop: Tank", group = "Teleop")
public class TeleopTank extends OpMode {

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMain robot = new HardwareMain();

    /**
     * Runs once when the OpMode is first enabled. The robot's hardware map is initialized.
     * @see OpMode#init()
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
    }


    /**
     * Runs continuously while OpMode is waiting to start.
     * @see OpMode#init_loop()
     */
    @Override
    public void init_loop() { }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * Runs continuously while the OpMode is active. Defines the driver-controlled actions
     * according to gamepad input.
     * @see OpMode#loop()
     */
    @Override
    public void loop() {
        // Adds runtime data to telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        // Drives the robot based on driver joystick input
        robot.drivetrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);

        // Clamps acquirer servos if triggers are pressed
        if (abs(gamepad1.left_trigger) > 0.1 || abs(gamepad2.left_trigger) > 0.1) {
            // Steps up the servo value until the max position is reached.
            robot.acquirer.clampIncrement(true);
        } else if (abs(gamepad1.right_trigger) > 0.1 || abs(gamepad2.right_trigger) > 0.1) {
            // Steps down the servo value until the min position is reached.
            robot.acquirer.clampIncrement(false);
        }

        // Moves slides if dpad up/down is pressed
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.acquirer.getSlides().setPower(-1);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.acquirer.getSlides().setPower(1);
        } else {
            robot.acquirer.getSlides().setPower(0);
        }

        // Toggles arm from upright and down positions if x or y is pressed
        if (gamepad1.x || gamepad2.x) {
            robot.arm.getArm().setPosition(0.1);
        } else if (gamepad1.y || gamepad2.y) {
            robot.arm.getArm().setPosition(1);
        }
    }
}
