package org.firstinspires.ftc.teamcode.opmode.auton.near;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

import java.util.ArrayList;

/**
 * AutonBlueGlyph is a class containing the following autonomous routine for the BLUE alliance:
 * <ol>
 *   <li>Score glyph into cryptobox</li>
 *   <li>Park in safe zone</li>
 * </ol>
 */
@Autonomous(name="Auton: Blue Glyph Near", group="Auton")
public class AutonBlueGlyph extends LinearOpMode {

    /* Robot hardware */
    private HardwareMain robot = new HardwareMain(this);

    /* Instance vars */
    private ArrayList<Integer> cols = new ArrayList<>();

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {

        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Wait until we're told to go
        robot.waitForStart();

        robot.arm.armUp();
        robot.scoreGlyphNear(1, false);
        cols.add(1);
        robot.scoreAdditionalGlyphsNear(cols, false);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
