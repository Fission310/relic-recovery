package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain;

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

        // Initialize robot
        robot.init(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        // Score jewel
//        double inches = robot.jewel(false);
//        telemetry.addData("Movement: ", inches);
//        telemetry.update();
//        sleep(1000);

        robot.drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 10, 10, 5);
        robot.drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 10, 10, 5);
        robot.drivetrain.turn(Drivetrain.TURN_SPEED, 90, 5);
        robot.drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -48, -48, 5);
        robot.drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 24, 24, 5);

    }

}
