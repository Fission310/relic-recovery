package org.firstinspires.ftc.teamcode.hardware;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain; // Change to current drivetrain
import org.firstinspires.ftc.teamcode.util.VisionManager;


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
    /**
     * Instance variable containing robot's relic.
     */
    public Relic relic;
    /**
     * Instance variable containing robot's flipper.
     */
    public Flipper flipper;

    /* Miscellaneous mechanisms */
    private ModernRoboticsI2cRangeSensor sensorDistance;

    /**
     * Default constructor for HardwareTank. Instantiates public mechanism instance variables.
     */
    public HardwareMain(){
        drivetrain = new Drivetrain();
        acquirer = new Acquirer();
        arm = new Arm();
        relic = new Relic();
        flipper = new Flipper();
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
        relic = new Relic(opMode);
        flipper = new Flipper(opMode);
    }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        acquirer.init(hwMap);
        arm.init(hwMap);
        relic.init(hwMap);
        flipper.init(hwMap);

        // Initialize range sensor
        sensorDistance = hwMap.get(ModernRoboticsI2cRangeSensor.class, RCConfig.RANGE_SENSOR);
    }

    /**
     * Autonomous action for scoring the jewel. Uses the robot's arm mechanism to detect jewel color
     * and moves forwards or backwards accordingly.
     * This assumes the color sensor faces the front of the bot.
     *
     *  @param visionManager    VisionManager containing the JewelDetector
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */
    public void jewel(VisionManager visionManager, boolean isAllianceRed) {

        // Run only if opMode is not stopped
        if (opMode.opModeIsActive()) {

            // Arm should lower to in between the jewels
            arm.sweeperNeutral();

            JewelDetector.JewelOrder jewelOrder = visionManager.getJewelOrder();
            opMode.telemetry.addData("Order: ", jewelOrder);
            opMode.telemetry.update();

            if (!jewelOrder.equals(JewelDetector.JewelOrder.UNKNOWN)) {
                if ((jewelOrder.equals(JewelDetector.JewelOrder.BLUE_RED) && isAllianceRed) || (jewelOrder.equals(JewelDetector.JewelOrder.RED_BLUE) && !isAllianceRed)) {
                    arm.sweeperLeft();
                } else {
                    arm.sweeperRight();
                }
            }

            arm.armDown();

            // Wait for arm to fully lower
            opMode.sleep(1000);

            arm.sweeperNeutral();
            opMode.sleep(500);
            arm.armUp();    // Move the arm back into upright position
            opMode.sleep(1000);

        }

    }

    /**
     * Autonomous action for scoring a glyph. Uses the robot's distance sensor to detect the robot's
     * position using the cryptobox wall. Moves parallel to cryptobox until the target column is
     * reached.
     * This assumes the distance sensor faces the cryptobox wall.
     *
     *  @param targetCol      the cryptobox column that is being targeted (left is 0, center is 1, right is 2)
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */
    public void scoreGlyph(int targetCol, boolean isAllianceRed) {

        int direction = isAllianceRed ? -1 : 1;
        relic.turnNeutral();
        drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, direction * 24, direction * 24, 5.0);
        drivetrain.turn(Drivetrain.TURN_SPEED, direction * 90, 10);
        drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -direction*7*(targetCol+1), -direction*7*(targetCol+1), 5.0);
        drivetrain.turn(Drivetrain.TURN_SPEED, 90, 10);
        drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -9, -9, 5.0);
        flipper.setLiftPower(-1);
        opMode.sleep(1000);
        flipper.setLiftPower(0);
        flipper.flipScore();
        drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 6, 6, 2.0);
        flipper.flipNeutral();
        flipper.setLiftPower(1);
        opMode.sleep(750);
        flipper.setLiftPower(0);
        drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, -6, -6, 2.0);
        drivetrain.driveToPos(Drivetrain.DRIVE_SPEED, 6, 6, 2.0);

    }

    public void scoreAdditionalGlyphs(int initialCol, boolean isAllianceRed) {

    }

}

