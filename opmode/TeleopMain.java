package org.firstinspires.ftc.teamcode.opmode;

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
 * A:               Toggles between flipping states: neutral and score
 * B:               Toggles flipper adjustment
 * Left bumper:     Toggle between relic turn score and neutral states
 * Right bumper:    Hold for slow mode
 * Left trigger:    Lower flipper lift
 * Right trigger:   Raise flipper lift
 * DPAD_UP:         Extend relic mechanism
 * DPAD_DOWN:       Retract relic mechanism
 * DPAD_LEFT:       Toggle relic clamp
 * DPAD_RIGHT:      Set relic turn to inside robot state
 * START:           Set arm position to up
 * BACK:            Toggles between sweeper states: left and right
 *
 */
@TeleOp(name = "Teleop: Main", group = "Teleop")
public class TeleopMain extends OpMode {

    /* CONSTANTS */
    private static final double ANALOG_THRESHOLD = 0.2;
    private static final double SLOW_MULTIPLIER = 0.5;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMain robot = new HardwareMain();

    /* Button debouncing */
    private boolean acquirerState, acquirerDebounce;
    private boolean flipState, flipDebounce;
    private boolean flipAdjustState, flipAdjustDebounce;
    private boolean clampState, clampDebounce;
    private boolean turnState, turnDebounce;        // turnState toggles between acquiring (true) and neutral (false) states
    private boolean sweeperState, sweeperDebounce;

    /**
     * Runs once when the OpMode is first enabled. The robot's hardware map is initialized.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
        acquirerState = false;
        acquirerDebounce = false;
        flipState = false;
        flipDebounce = false;
        flipAdjustState = false;
        flipAdjustDebounce = false;
        clampState = true;
        clampDebounce = false;
        turnState = false;
        turnDebounce = false;
        sweeperState = false;
        sweeperDebounce = false;
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
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        robot.arm.armUp();
        robot.arm.sweeperNeutral();
        robot.relic.turnAcq();

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

        // Threshold for strafing, makes horizontal strafing easier
        double yInput = gamepad1.left_stick_y;
        if (abs(gamepad1.left_stick_y) < ANALOG_THRESHOLD) {
            yInput = 0;
        }
        // Drives the robot based on driver joystick input, check for slow mode
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.drivetrain.drive(gamepad1.left_stick_x * SLOW_MULTIPLIER, yInput * SLOW_MULTIPLIER, gamepad1.right_stick_x * SLOW_MULTIPLIER);
        } else {
            robot.drivetrain.drive(gamepad1.left_stick_x, yInput, gamepad1.right_stick_x);
        }

        // Set arm position to up
        if (gamepad1.start || gamepad2.start) {
            robot.arm.armUp();
        }

        // Toggles between flipping states: neutral and score
        if (gamepad1.a || gamepad2.a) {
            if (!flipDebounce) {
                flipState = !flipState;
                if (flipState) {
                    robot.flipper.flipScore();
                } else {
                    robot.flipper.flipNeutral();
                }
                flipDebounce = true;
            }
        } else {
            flipDebounce = false;
        }

        // Toggles between acquirer adjustment state
        if (gamepad1.b || gamepad2.b) {
            if (!flipAdjustDebounce) {
                flipAdjustState = !flipAdjustState;
                if (flipAdjustState) {
                    robot.flipper.flipAdjust();
                } else {
                    robot.flipper.flipNeutral();
                }
                flipAdjustDebounce = true;
            }
        } else {
            flipAdjustDebounce = false;
        }

        // Toggles acquirer
        if (gamepad1.x || gamepad2.x) {
            if (!acquirerDebounce) {
                acquirerState = !acquirerState;
                robot.acquirer.setIntakePower(acquirerState ? -1 : 0);
                acquirerDebounce = true;
            }
        } else if (gamepad1.y || gamepad2.y) {
            acquirerState = true;
            robot.acquirer.setIntakePower(1);
        } else {
            acquirerDebounce = false;
        }

        // Lower and raise flipper lift
        if (gamepad1.left_trigger > ANALOG_THRESHOLD || gamepad2.left_trigger > ANALOG_THRESHOLD) {
            double total_left_trigger = gamepad1.left_trigger + gamepad2.left_trigger;
            robot.flipper.setLiftPower(total_left_trigger > 1 ? gamepad1.left_trigger : total_left_trigger);
        } else if (gamepad1.right_trigger > ANALOG_THRESHOLD || gamepad2.right_trigger > ANALOG_THRESHOLD) {
            double total_right_trigger = gamepad1.right_trigger + gamepad2.right_trigger;
            robot.flipper.setLiftPower(total_right_trigger > 1 ? -gamepad1.right_trigger : -total_right_trigger);
        } else {
            robot.flipper.setLiftPower(0);
        }

        // Moves slides if dpad up/down is pressed
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.relic.setSlidesPower(1);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.relic.setSlidesPower(-1);
        } else {
            robot.relic.setSlidesPower(0);
        }

        // Toggle relic clamp
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            if (!clampDebounce) {
                clampState = !clampState;
                if (clampState) {
                    robot.relic.clamp();
                } else {
                    robot.relic.unclamp();
                }
                clampDebounce = true;
            }
        } else {
            clampDebounce = false;
        }

        // Set relic turn states
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            if (!turnDebounce) {
                turnState = !turnState;
                if (turnState) {
                    // acquiring
                    robot.relic.turnAcq();
                } else {
                    // neutral
                    robot.relic.turnNeutral();
                }
                turnDebounce = true;
            }
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            turnState = true;
            robot.relic.turnInside();
        } else {
            turnDebounce = false;
        }

        // Sweeper states
        if (gamepad1.back || gamepad2.back) {
            if (!sweeperDebounce) {
                sweeperState = !sweeperState;
                if (sweeperState) {
                    robot.arm.sweeperRight();
                } else {
                    robot.arm.sweeperLeft();
                }
                sweeperDebounce = true;
            }
        } else {
            sweeperDebounce = false;
        }

        double[] positions = robot.drivetrain.getPositions();
        telemetry.addData("Path2", "Running at %.2f :%.2f :%.2f :%.2f",
                positions[0],
                positions[1],
                positions[2],
                positions[3]);

    }
}
