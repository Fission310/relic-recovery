package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name="Auton: Red Jewel", group="Auton")
public class AutonRedJewel extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    private HardwareMain robot = new HardwareMain(this);

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        // Clamp onto glyph and init top servos
        //robot.clamp(this);

        robot.jewel(true);

    }

}
