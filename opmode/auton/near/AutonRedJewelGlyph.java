package org.firstinspires.ftc.teamcode.opmode.auton.near;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.util.VisionManager;

/**
 * AutonRedJewelGlyph is a class containing the following autonomous routine for the RED alliance:
 * <ol>
 *   <li>Score jewel</li>
 *   <li>Score glyph in key</li>
 *   <li>Park in safe zone</li>
 * </ol>
 */
@Autonomous(name="Auton: Red Jewel Glyph Near", group="Auton")
public class AutonRedJewelGlyph extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareMain robot = new HardwareMain(this);

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {

        // Initialize CV
        VisionManager visionManager = new VisionManager();
        visionManager.vuforiaInit(hardwareMap);

        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Wait until we're told to go
        waitForStart();

        runtime.reset();
        int key = -1;
        while (opModeIsActive() && key == -1 && runtime.seconds() < 3) {
            key = visionManager.getKey();
        }
        if (key == -1) { key = 1; }

        telemetry.addData("Key:", key);
        telemetry.update();
        visionManager.vuforiaStop();

        visionManager.jewelInit(hardwareMap);
        sleep(2000);

        // Score jewel
        robot.jewel(visionManager, true);

        // Score glyph
        robot.scoreGlyphNear(key, true);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Stop CV
        visionManager.jewelStop();

    }

}
