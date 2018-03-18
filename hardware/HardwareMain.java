package org.firstinspires.ftc.teamcode.hardware;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain;
import org.firstinspires.ftc.teamcode.util.VisionManager;

import java.util.ArrayList;

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
     * Default constructor for HardwareMain. Instantiates public mechanism instance variables.
     */
    public HardwareMain(){
        drivetrain = new Drivetrain();
        acquirer = new Acquirer();
        arm = new Arm();
        relic = new Relic();
        flipper = new Flipper();
    }
    /**
     * Overloaded constructor for HardwareMain. Calls the default constructor and sets the OpMode
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
     * Waits for opMode's to start. Can perform actions while waiting.
     */
    public void waitForStart() {
        while (!opMode.isStarted()) {
            opMode.telemetry.addData("Heading:", drivetrain.getHeading());
            opMode.telemetry.update();
        }
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
        drivetrain.driveToPos(-direction * 21 - 3, 5.0);
        drivetrain.turn(-direction * 90, 10);
        drivetrain.driveToPos(-direction * CRYPTO_COLUMN_WIDTH*(targetCol+1), 5.0);
        drivetrain.turn(-(direction - 1) * 90, 10); // 0 IF BLUE, 180 IF RED

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
        drivetrain.driveToPos(-direction * (17 + CRYPTO_COLUMN_WIDTH*(targetCol+1)), 5.0);
        drivetrain.turn(-90, 5.0);
        drivetrain.driveToPos(-6, 2.0);

        // score glyph
        flipAndAlignGlyph();

    }

    // Routine for scoring and aligning the glyph in the cryptobox
    private void flipAndAlignGlyph() {
        flipper.flipScore();
        opMode.sleep(500);
        drivetrain.driveToPos(-6, 2.0);
        flipper.flipNeutral();;
        drivetrain.driveToPos(6, 2.0);
        drivetrain.driveToPos(-6, 2.0);
        drivetrain.driveToPos(6, 2.0);
    }

    /**
     * Assumes that robot is in the safe zone after flipping and aligning. Finds and scores additional
     * glyphs into a different column.
     * @param initialCols
     * @param isAllianceRed
     * @return                  column scored in
     */
    public int scoreAdditionalGlyphsFar(ArrayList<Integer> initialCols, boolean isAllianceRed) {

        // relative coordinates to desired point in glyph pit
        double x0 = 48 - CRYPTO_COLUMN_WIDTH*initialCols.get(initialCols.size() - 1);
        double y0 = 48;

        // rotate to desired point
        drivetrain.turn(-Math.toDegrees(Math.atan(x0/y0)), 2.0);
        // drive to the desired point
        drivetrain.driveToPos(Math.sqrt(x0*x0 + y0*y0), 5.0);

        acquirer.setIntakePower(1);
        opMode.sleep(2000);
        acquirer.setIntakePower(0);

        // determine target cryptobox column
        int targetCol;
        if (initialCols.indexOf(2) != -1) {
            targetCol = 2;
        } else if (initialCols.indexOf(1) != -1) {
            targetCol = 1;
        } else if (initialCols.indexOf(0) != -1) {
            targetCol = 0;
        } else {
            initialCols.clear();
            targetCol = 2;
        }

        double x1 = 48 - CRYPTO_COLUMN_WIDTH*targetCol;
        double y1 = -48;

        // rotate to desired column
        drivetrain.turn(0, 2.0);
        drivetrain.turn(-Math.toDegrees(Math.atan(x1/y1)), 2.0);
        // drive to the desired point
        drivetrain.driveToPos(-Math.sqrt(x1*x1 + y1*y1), 5.0);
        // turn to initial heading
        drivetrain.turn(0, 2.0);

        flipAndAlignGlyph();

        return targetCol;

    }

    /**
     * Assumes that robot is in the safe zone after flipping and aligning. Finds and scores additional
     * glyphs into a different column.
     * @param initialCols
     * @param isAllianceRed
     * @return                  column scored in
     */
    public int scoreAdditionalGlyphsNear(ArrayList<Integer> initialCols, boolean isAllianceRed) {

        drivetrain.driveToPos(-24, 5.0);

        acquirer.setIntakePower(1);
        opMode.sleep(2000);
        acquirer.setIntakePower(0);

        // determine target cryptobox column
        int targetCol;
        if (initialCols.indexOf(2) != -1) {
            targetCol = 2;
        } else if (initialCols.indexOf(1) != -1) {
            targetCol = 1;
        } else if (initialCols.indexOf(0) != -1) {
            targetCol = 0;
        } else {
            initialCols.clear();
            targetCol = 2;
        }

        double x1 = CRYPTO_COLUMN_WIDTH*(targetCol-1);
        double y1 = 24;

        // rotate to desired column
        drivetrain.turn(Math.toDegrees(Math.atan(x1/y1)), 2.0);
        opMode.sleep(1500);
        // drive to the desired point
        drivetrain.driveToPos(Math.sqrt(x1*x1 + y1*y1), 5.0);
        opMode.sleep(1500);
        // turn to initial heading
        drivetrain.turn(0, 2.0);
        opMode.sleep(1500);

        flipAndAlignGlyph();

        return targetCol;

    }

    public void findGlyph(VisionManager visionManager) {
        while (opMode.opModeIsActive()) {
            opMode.telemetry.addData("Glyph Pos X:", visionManager.getGlyphPosX());
            opMode.telemetry.addData("Glyph Pos Y:", visionManager.getGlyphPosY());
            opMode.telemetry.addData("Glyph Offset:", visionManager.getGlyphOffset());
            opMode.telemetry.update();
        }
    }

}

