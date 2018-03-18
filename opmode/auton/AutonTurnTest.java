package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

import java.util.ArrayList;

@Autonomous(name="Test: Turn Test", group="Auton")
public class AutonTurnTest extends LinearOpMode {

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
        robot.drivetrain.turn(-90, 5.0);
        sleep(1000);
        robot.drivetrain.turn(90, 5.0);
        sleep(1000);
        robot.drivetrain.turn(0, 5.0);
        sleep(1000);
        robot.drivetrain.turn(180, 5.0);
        sleep(1000);
        robot.drivetrain.turn(45, 5.0);
        sleep(1000);
        robot.drivetrain.turn(-45, 5.0);
        sleep(1000);
        robot.drivetrain.turn(170, 5.0);
        sleep(1000);
        robot.drivetrain.turn(-170, 5.0);
        sleep(1000);
        robot.drivetrain.turn(20, 5.0);
        robot.drivetrain.turn(-20, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
