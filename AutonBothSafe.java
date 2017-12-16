package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

/**
 * AutonBothSafe is a class containing the following autonomous routine for BOTH alliances:
 * <ol>
 *   <li>Drive straight and park in safe zone</li>
 * </ol>
 */
@Autonomous(name="Auton: Both Safe", group="Auton")
@Disabled
public class AutonBothSafe extends LinearOpMode {

    /* Robot hardware */
    private HardwareMain robot = new HardwareMain(this);

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {

        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Wait until we're told to go
        waitForStart();
        robot.drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 48, 48, 5.0);

        // Note: can extend arm at end to ensure safe zone park

        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
