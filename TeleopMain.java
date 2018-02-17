package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

import static java.lang.Math.abs;

/**
 * TeleopMain is the primary TeleOp OpMode for Relic Recovery. All driver-controlled actions should
 * be defined in this class.
 *
 * BUTTON MAPPINGS:
 * Left stick:      Control robot's velocity and direction
 * Right stick x:   Turn robot
 * X:               Turn intake on/off
 * Y:               Expel glyphs from intake
 * -A:              Toggles between flipping states: acquiring, neutral, and score
 * -B:              Set flipper to acquiring state
 * -Left bumper:    Toggle between relic turn score and neutral states
 * -Right bumper:   (slow mode?)
 * -Left trigger:   Lower flipper lift
 * -Right trigger:  Raise flipper lift
 * DPAD_UP:         Extend relic mechanism
 * DPAD_DOWN:       Retract relic mechanism
 * -DPAD_LEFT:      Toggle relic clamp
 * -DPAD_RIGHT:     Set relic turn to inside robot state
 * START:           Toggle arm position
 * -BACK:
 *
 */
@TeleOp(name = "Teleop: Main", group = "Teleop")
public class TeleopMain extends OpMode {

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMain robot = new HardwareMain();

    /* Button debouncing */
    private boolean acquirerState, acquirerDebounce;
    private boolean armState, armDebounce;

    /**
     * Runs once when the OpMode is first enabled. The robot's hardware map is initialized.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        acquirerState = false;
        acquirerDebounce = false;
        armState = false;
        armDebounce = false;
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
            if (!armDebounce) {
                armState = !armState;
                robot.arm.setArmPosition(armState ? 0.1 : 1);
                armDebounce = true;
            }
        } else {
            armDebounce = false;
        }

        // Toggles turn servo from up or down positions if a or b is pressed
        if (gamepad1.a || gamepad2.a) {
            robot.relic.turnDown();
        } else if (gamepad1.b || gamepad2.b) {
            robot.relic.turnInside();
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.relic.turnInside();
        }

        // Toggles clamp servo from up or down positions if x or y is pressed
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.relic.clamp();
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            robot.relic.unclamp();
        }

        // Toggles acquirer
        if (gamepad1.x || gamepad2.x) {
            if (!acquirerDebounce) {
                acquirerState = !acquirerState;
                robot.acquirer.setIntakePower(acquirerState ? 1 : 0);
                acquirerDebounce = true;
            }
        } else if (gamepad1.y || gamepad2.y) {
            acquirerState = true;
            robot.acquirer.setIntakePower(-1);
        } else {
            acquirerDebounce = false;
        }
    }
}
