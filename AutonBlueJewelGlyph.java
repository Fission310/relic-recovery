package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain;
import org.firstinspires.ftc.teamcode.util.VisionManager;

/**
 * AutonBlueJewelGlyph is a class containing the following autonomous routine for the BLUE alliance:
 * <ol>
 *   <li>Score jewel</li>
 *   <li>Score glyph</li>
 *   <li>Park in safe zone</li>
 * </ol>
 */
@Autonomous(name="Auton: Blue Jewel Glyph", group="Auton")
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
        waitForStart();

        int key = -1;
        while (opModeIsActive() && key == -1) {
            key = visionManager.getKey();
        }

        telemetry.addData("Key:", key);
        telemetry.update();
        visionManager.vuforiaStop();

        visionManager.jewelInit(hardwareMap);

        // Score jewel
        robot.jewel(visionManager, true);
        sleep(1000);

        // Score glyph
        robot.scoreGlyph(key, false);

        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Stop CV
        visionManager.jewelStop();

    }

}
