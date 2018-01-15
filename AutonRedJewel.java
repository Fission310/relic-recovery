package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.util.VisionManager;

/**
 * AutonBlueJewel is a class containing the following autonomous routine for the RED alliance:
 * <ol>
 *   <li>Score jewel</li>
 * </ol>
 */
@Autonomous(name="Auton: Red Jewel", group="Auton")
public class AutonRedJewel extends LinearOpMode {

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
        visionManager.jewelInit(hardwareMap);

        // Initialize robot
        robot.init(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        // Score jewel
        double inches = robot.jewel(visionManager, true);
        telemetry.addData("Movement: ", inches);
        telemetry.update();
        sleep(1000);

        // Stop CV
        visionManager.jewelStop();
    }

}
