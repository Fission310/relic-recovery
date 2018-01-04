package org.firstinspires.ftc.teamcode.hardware.tankdrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

/**
 * HardwareTank is the class that is used to define all of the hardware for a single robot. In this
 * case, the robot is: Relic Recovery.
 *
 * This class contains autonomous actions involving multiple mechanisms. These actions
 * may be common to more than one routine.
 */
public class HardwareTank extends Mechanism {

    /* Mechanisms */
    /**
     * Instance variable containing robot's drivetrain.
     */
    public Drivetrain drivetrain;
    /**
     * Instance variable containing robot's acquirer.
     */
    public Acquirer acquirer;
    /**
     * Instance variable containing robot's arm.
     */
    public Arm arm;

    /* Miscellaneous mechanisms */
    DistanceSensor sensorDistance;

    /**
     * Default constructor for HardwareTank. Instantiates public mechanism instance variables.
     */
    public HardwareTank(){
        drivetrain = new Drivetrain();
        acquirer = new Acquirer();
        arm = new Arm();
    }
    /**
     * Overloaded constructor for HardwareTank. Calls the default constructor and sets the OpMode
     * context for the robot.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public HardwareTank(LinearOpMode opMode){
        this.opMode = opMode;
        drivetrain = new Drivetrain(opMode);
        acquirer = new Acquirer(opMode);
        arm = new Arm(opMode);
    }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        acquirer.init(hwMap);
        arm.init(hwMap);

        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");
    }

    /**
     * Autonomous action for scoring the jewel. Uses the robot's arm mechanism to detect jewel color
     * and moves forwards or backwards accordingly.
     * This assumes the color sensor faces the front of the bot.
     *
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     *  @return                 number of inches moved, with respect to the original position
     */
    public int jewel(boolean isAllianceRed) {

        // Run only if opMode is not stopped
        if (opMode.opModeIsActive()) {

            // Arm should lower to in between the jewels
            arm.getArm().setPosition(0.1);

            // Get test hue values and update telemetry
            float[] hsvValues = arm.getHSVValues();
            opMode.telemetry.addData("HSV: ", hsvValues);
            opMode.telemetry.update();

            // Wait for arm to fully lower
            opMode.sleep(1000);

            // Number of inches of driving required to knock the jewel off
            int inchesToDrive = 5;

            // Get hue values to use
            hsvValues = arm.getHSVValues();

            // Checks if hue value is greater than a threshold value indicating blue
            if (hsvValues[0] > Arm.BLUE) {

                // Moves forwards or backwards based on alliance color
                if (isAllianceRed) {
                    // forwards
                    drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -inchesToDrive, -inchesToDrive, 2);
                    arm.getArm().setPosition(1);    // Move the arm back into upright position
                    return inchesToDrive;
                } else {
                    // backwards
                    drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, inchesToDrive, inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    return -inchesToDrive;
                }
            }
            // Checks if hue value is less than a threshold value indicating red
            else if (hsvValues[0] < Arm.RED) {

                // Moves forwards or backwards based on alliance color
                if (isAllianceRed) {
                    // backwards
                    drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, inchesToDrive, inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    return inchesToDrive;
                } else {
                    // forwards
                    drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -inchesToDrive, -inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    return -inchesToDrive;
                }
            }
        }

        // If the hue does not pass the threshold test, move the arm back into upright position
        arm.getArm().setPosition(1);
        return 0;
    }


    /**
     * Autonomous action for scoring a glyph. Uses the robot's distance sensor to detect the robot's
     * position using the cryptobox wall. Moves parallel to cryptobox until the target column is
     * reached.
     * This assumes the distance sensor faces the cryptobox wall.
     *
     *  @param targetCol    the cryptobox column that is being targeted (left is 0, center is 1, right is 2)
     */
    public void scoreGlyph(int targetCol) {

        // Number of cryptobox walls detected
        int wallsDetected = -1;

        // Continue moving parallel to cryptobox wall until the target column is reached
        while (opMode.opModeIsActive() && wallsDetected < targetCol) {

            // Get the distance from the cryptobox or cryptobox walls
            double boxDistance = sensorDistance.getDistance(DistanceUnit.CM);

            // Check if a wall is being detected
            if (boxDistance < 10) {
                wallsDetected++;
            }
        }

        // Score the glyph after the target has been reached
    }

 }

