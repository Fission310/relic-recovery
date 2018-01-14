package org.firstinspires.ftc.teamcode.hardware;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Change depending on Drivetrain used
import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain;



/**
 * HardwareTank is the class that is used to define all of the hardware for a single robot. In this
 * case, the robot is: Relic Recovery.
 *
 * This class contains autonomous actions involving multiple mechanisms. These actions
 * may be common to more than one routine.
 */
public class HardwareMain extends Mechanism {

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
    ModernRoboticsI2cRangeSensor sensorDistance;
    CryptoboxDetector cryptoboxDetector;

    /**
     * Default constructor for HardwareTank. Instantiates public mechanism instance variables.
     */
    public HardwareMain(){
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
    public HardwareMain(LinearOpMode opMode){
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

        // Initialize range sensor
        sensorDistance = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

//        // Initialize CV
//        cryptoboxDetector = new CryptoboxDetector();
//        cryptoboxDetector.init(hwMap.appContext, CameraViewDisplay.getInstance());
//
//        cryptoboxDetector.rotateMat = false;
//
//        //Optional Test Code to load images via Drawables
//        //cryptoboxDetector.useImportedImage = true;
//        //cryptoboxDetector.SetTestMat(com.qualcomm.ftcrobotcontroller.R.drawable.test_cv4);
//
//        cryptoboxDetector.enable();
    }

    public void stop() {
        //cryptoboxDetector.disable();
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

            // Get test hue values and update telemetry
            float[] hsvValues;

            // Number of inches of driving required to knock the jewel off
            int inchesToDrive = 10;

            // Get hue values to use
            // hsvValues = arm.getHSVValues();
            // opMode.telemetry.addData("HSV: ", hsvValues[0]);
            // opMode.telemetry.update();
            // opMode.sleep(1000);
            opMode.sleep(3000);
            JewelDetector.JewelOrder jewelOrder = arm.getJewelOrder();
            opMode.telemetry.addData("Order: ", jewelOrder);
            opMode.telemetry.update();

            // Arm should lower to in between the jewels
            arm.getArm().setPosition(0);

            // Wait for arm to fully lower
            opMode.sleep(1000);


            // Checks if hue value is greater than a threshold value indicating blue
            if (jewelOrder.equals(JewelDetector.JewelOrder.BLUE_RED)) {

                // Moves forwards or backwards based on alliance color
                if (isAllianceRed) {
                    // backwards
                    drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -inchesToDrive, -inchesToDrive, 2);
                    arm.getArm().setPosition(1);    // Move the arm back into upright position
                    opMode.sleep(1000);
                    arm.stop();
                    return -inchesToDrive;
                } else {
                    // forwards
                    drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, inchesToDrive, inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    opMode.sleep(1000);
                    arm.stop();
                    return inchesToDrive;
                }
            }
            // Checks if hue value is less than a threshold value indicating red
            else if (jewelOrder.equals(JewelDetector.JewelOrder.RED_BLUE)) {

                // Moves forwards or backwards based on alliance color
                if (isAllianceRed) {
                    // forwards
                    drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, inchesToDrive, inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    opMode.sleep(1000);
                    arm.stop();
                    return inchesToDrive;
                } else {
                    // backwards
                    drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -inchesToDrive, -inchesToDrive, 2);
                    arm.getArm().setPosition(1);
                    opMode.sleep(1000);
                    arm.stop();
                    return -inchesToDrive;
                }
            }

            // If the hue does not pass the threshold test, move the arm back into upright position
            arm.getArm().setPosition(1);
            opMode.sleep(1000);
            arm.stop();
            return 0;
        }

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
        while (opMode.opModeIsActive()) {
            opMode.telemetry.addData("isCryptoBoxDetected", cryptoboxDetector.isCryptoBoxDetected());
            opMode.telemetry.addData("isColumnDetected ",  cryptoboxDetector.isColumnDetected());

            opMode.telemetry.addData("Column Left ",  cryptoboxDetector.getCryptoBoxLeftPosition());
            opMode.telemetry.addData("Column Center ",  cryptoboxDetector.getCryptoBoxCenterPosition());
            opMode.telemetry.addData("Column Right ",  cryptoboxDetector.getCryptoBoxRightPosition());
            opMode.telemetry.update();
            // center is always 190 +- 10

            int imageCenter = 180;
            int threshold = 10;

            if (cryptoboxDetector.isCryptoBoxDetected()) {
                int centerPos = cryptoboxDetector.getCryptoBoxCenterPosition();
                while (opMode.opModeIsActive() &&
                        cryptoboxDetector.isCryptoBoxDetected() &&
                        Math.abs(centerPos - imageCenter) < threshold) {
                    if (centerPos > imageCenter + threshold) {
                        drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 1, 1, 1);
                    } else {
                        drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -1, -1, 1);
                    }
                    centerPos = cryptoboxDetector.getCryptoBoxCenterPosition();
                }

                // Turn and score
                if (Math.abs(centerPos - imageCenter) < threshold) {

                }

            } else {
                drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 1, 1, 1);
            }

        }

        // Number of cryptobox walls detected
//        int wallsDetected = -1;
//
//        // Continue moving parallel to cryptobox wall until the target column is reached
//        while (opMode.opModeIsActive() && wallsDetected < targetCol) {
//
//            // Get the distance from the cryptobox or cryptobox walls
//            double boxDistance = sensorDistance.getDistance(DistanceUnit.CM);
//
//            // Check if a wall is being detected
//            if (boxDistance < 10) {
//                wallsDetected++;
//            }
//
//            drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 1, 1, 1);
//        }
//
//        // Score the glyph after the target has been reached
    }

 }

