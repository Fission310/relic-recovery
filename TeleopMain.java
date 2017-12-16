package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

import static java.lang.Math.abs;

/**
 *
 */
@TeleOp(name = "Teleop: Main", group = "Teleop")
public class TeleopMain extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private HardwareMain robot = new HardwareMain();

    @Override
    public void init() {
        robot.init(hardwareMap);
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

        robot.drivetrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);

        if (abs(gamepad1.left_trigger) > 0.1 || abs(gamepad2.left_trigger) > 0.1) {
            // Keep stepping up until we hit the max value.
            robot.acquirer.clampIncrement(true);

        }
        else if (abs(gamepad1.right_trigger) > 0.1 || abs(gamepad2.right_trigger) > 0.1) {
            // Keep stepping down until we hit the min value.
            robot.acquirer.clampIncrement(false);
        }

        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.acquirer.getSlides().setPower(-1);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.acquirer.getSlides().setPower(1);
        } else {
            robot.acquirer.getSlides().setPower(0);
        }

        if (gamepad1.x || gamepad2.x) {
            robot.arm.getArm().setPosition(0.1);
        } else if (gamepad1.y || gamepad2.y) {
            robot.arm.getArm().setPosition(1);
        }
    }
}
