package org.firstinspires.ftc.teamcode.hardware;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain; // Change to current drivetrain
import org.firstinspires.ftc.teamcode.util.VisionManager;

import static org.firstinspires.ftc.teamcode.FieldConstants.CRYPTO_COLUMN_WIDTH;


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

            arm.sweeperNeutral();

            JewelDetector.JewelOrder jewelOrder = visionManager.getJewelOrder();
            opMode.telemetry.addData("Order: ", jewelOrder);
            opMode.telemetry.update();

            arm.armDown();

            // Wait for arm to fully lower
            opMode.sleep(750);

            if (!jewelOrder.equals(JewelDetector.JewelOrder.UNKNOWN)) {
                if ((jewelOrder.equals(JewelDetector.JewelOrder.BLUE_RED) && isAllianceRed) || (jewelOrder.equals(JewelDetector.JewelOrder.RED_BLUE) && !isAllianceRed)) {
                    arm.sweeperLeft();
                } else {
                    arm.sweeperRight();
                }
            }

            opMode.sleep(750);

            arm.sweeperNeutral();
            arm.armUp();    // Move the arm back into upright position
            opMode.sleep(1000);

        }

    }

    /**
     * Autonomous action for scoring a glyph. Uses the robot's distance sensor to detect the robot's
     * position using the cryptobox wall. Moves parallel to cryptobox until the target column is
     * reached.
     * This assumes the target is the far cryptobox.
     *
     *  @param targetCol      the cryptobox column that is being targeted (left is 0, center is 1, right is 2)
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */
    public void scoreGlyphFar(int targetCol, boolean isAllianceRed) {

        int direction = isAllianceRed ? -1 : 1;
        relic.turnNeutral();

        // drive to position
        drivetrain.driveToPos(-direction * 24, 5.0);
        drivetrain.turn(-direction * 90, 10);
        drivetrain.driveToPos(direction * (3 + CRYPTO_COLUMN_WIDTH*(targetCol+1)), 5.0);
        drivetrain.turn(0, 10);
        drivetrain.driveToPos(9, 5.0);

        // score glyph
        flipAndAlignGlyph();

    }

    /**
     * Autonomous action for scoring a glyph. Uses the robot's distance sensor to detect the robot's
     * position using the cryptobox wall. Moves parallel to cryptobox until the target column is
     * reached.
     * This assumes the target is the near cryptobox.
     *
     *  @param targetCol      the cryptobox column that is being targeted (left is 0, center is 1, right is 2)
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */
    public void scoreGlyphNear(int targetCol, boolean isAllianceRed) {

        int direction = isAllianceRed ? -1 : 1;
        relic.turnNeutral();

        // drive to position
        drivetrain.driveToPos(direction * (17 + CRYPTO_COLUMN_WIDTH*(targetCol+1)), 5.0);
        drivetrain.turn(90, 5.0);
        drivetrain.driveToPos(9, 2.0);

        // score glyph
        flipAndAlignGlyph();

    }

    // Routine for scoring and aligning the glyph in the cryptobox
    private void flipAndAlignGlyph() {
        flipper.setLiftPower(-1);
        opMode.sleep(500);
        flipper.setLiftPower(0);
        flipper.flipScore();
        drivetrain.driveToPos(-6, 2.0);
        opMode.sleep(500);
        flipper.flipNeutral();
        flipper.setLiftPower(1);
        opMode.sleep(250);
        flipper.setLiftPower(0);
        drivetrain.driveToPos(6, 2.0);
        drivetrain.driveToPos(-6, 2.0);
    }

    public void scoreAdditionalGlyphs(int initialCol, boolean isAllianceRed) {

    }


    public void findGlyph(VisionManager visionManager) {
        while (opMode.opModeIsActive()) {
            opMode.telemetry.addData("Glyph Pos X:", visionManager.getGlyphPosX());
            opMode.telemetry.addData(                                                                        "Glyph Pos Y:", visionManager.getGlyphPosY());
            opMode.telemetry.addData("Glyph Offset:", visionManager.getGlyphOffset());
            opMode.telemetry.update();
        }
    }

}

