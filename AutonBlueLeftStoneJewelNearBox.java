package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auton: Blue Jewel Near Cryptobox @ Left Stone", group="Auton")
@Disabled
public class AutonBlueLeftStoneJewelNearBox extends LinearOpMode {

    private HardwareMain robot = new HardwareMain(this);

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Wait until we're told to go
        waitForStart();
        robot.drivetrain.encoderDrive(this, Drivetrain.DRIVE_SPEED, 48, 48, 5.0);
        robot.arm.getArm().setPosition(0.1);

        // Clamp onto glyph and init top servos
        //robot.clamp(this);
        //sleep(1000);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        int jewelMovement;

        //jewelMovement = robot.jewel(this, false);
        //sleep(1000);
//        robot.encoderDrive(this, HardwareMain.DRIVE_SPEED,  48 + jewelMovement,  48 + jewelMovement, 5.0);
//        sleep(1000);
//        robot.turn(this, HardwareMain.TURN_SPEED,  -90, 2.0);
//        sleep(1000);
//        robot.encoderDrive(this, HardwareMain.DRIVE_SPEED,  24,  24, 5.0);

        // interesting note: can extend arm at end to ensure safe zone park
        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
