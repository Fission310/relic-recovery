package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.util.VisionManager;

/**
 * Autonomous program for testing glyph detection.
 */
@Autonomous(name="Auton: Glyph Detector", group="Auton")
public class AutonGlyphDetector extends LinearOpMode {

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
        visionManager.glyphInit(hardwareMap);

        // Initialize robot
        robot.init(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        // Score jewel
        robot.findGlyph(visionManager);

        // Stop CV
        visionManager.glyphStop();

    }

}
