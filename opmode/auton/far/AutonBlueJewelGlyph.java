package org.firstinspires.ftc.teamcode.opmode.auton.far;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.util.VisionManager;

/**
 * AutonBlueJewelGlyph is a class containing the following autonomous routine for the BLUE alliance:
 * <ol>
 *   <li>Score jewel</li>
 *   <li>Score glyph in key</li>
 *   <li>Park in safe zone</li>
 * </ol>
 */
@Autonomous(name="Auton: Blue Jewel Glyph Far", group="Auton")
public class AutonBlueJewelGlyph extends LinearOpMode {

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
        robot.waitForStart();

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
        robot.jewel(visionManager, false);

        // Score glyph
        robot.scoreGlyphFar(key, false);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Stop CV
        visionManager.jewelStop();

    }

}
