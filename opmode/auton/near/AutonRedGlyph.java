package org.firstinspires.ftc.teamcode.opmode.auton.near;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

import java.util.ArrayList;

/**
 * AutonBlueGlyph is a class containing the following autonomous routine for the RED alliance:
 * <ol>
 *   <li>Score glyph into cryptobox</li>
 *   <li>Park in safe zone</li>
 * </ol>
 */
@Autonomous(name="Auton: Red Glyph Near", group="Auton")
public class AutonRedGlyph extends LinearOpMode {

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
//        robot.drivetrain.turn(-90, 5.0);
//        robot.drivetrain.turn(90, 5.0);
//        robot.drivetrain.turn(0, 5.0);
//        robot.drivetrain.turn(180, 5.0);

        robot.scoreGlyphNear(1, true);
        //cols.add(1);
        //robot.scoreAdditionalGlyphsNear(cols, true);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
