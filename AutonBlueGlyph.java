package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain;

/**
 * AutonBlueGlyph is a class containing the following autonomous routine for the BLUE alliance:
 * <ol>
 *   <li>Score glyph into cryptobox</li>
 *   <li>Park in safe zone</li>
 * </ol>
 */
@Autonomous(name="Auton: Blue Glyph", group="Auton")
public class AutonBlueGlyph extends LinearOpMode {

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
        robot.scoreGlyph(1, false);

        // Note: can extend arm at end to ensure safe zone park

        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
