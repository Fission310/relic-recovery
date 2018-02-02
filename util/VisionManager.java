package org.firstinspires.ftc.teamcode.util;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VisionManager {

    private JewelDetector jewelDetector;
    private ClosableVuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    public void jewelInit(HardwareMap hwMap) {
        // Initialize CV
        jewelDetector = new JewelDetector();
        jewelDetector.init(hwMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;

        jewelDetector.enable();
    }

    public void jewelStop() {
        jewelDetector.disable();
    }

    /**
     * Get the order of Jewels using DogeCV.
     * @return      BLUERED, REDBLUE, or UNDECIDED based on jewel order
     */
    public JewelDetector.JewelOrder getJewelOrder() {
        return jewelDetector.getCurrentOrder();
    }

    public void vuforiaInit(HardwareMap hwMap) {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        ClosableVuforiaLocalizer.Parameters parameters = new ClosableVuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "ATHlAS7/////AAAAGSrlghNuCkIdu0Y/Eqnxz9oejoRzibKYqWJYEJik+9ImrFuJaDs2/WAm5ovuC4iV/m4DHM3WWgAl9pI5MQULOsKslna/+bYWzcbzpzak4NMtWuGLnnJYCeH8vP2x8fC8R0I+Odvd4vhnJdSa3P6C87oTqtVSX0sZcVOvALmUpCJcSFHAqshW0F7XziW89qM4tBDQoKgNCkbFNmKeRnKa4j4Vfyk0RSNXc/79shIk8Pu4j8krsBComGYTx4FKsClnfgZYOp51uhMg/yoEHfpy0XMrCOBZUYIyTVvOsCtC9GzLAOLoxEnunRRjagCKni32kkrH07slhuiCqpNBJQ02y8qZFChTjt5i+ZZwnzaWCFSf";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = ClosableVuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public int getKey() {
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            return 0;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            return 1;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            return 2;
        } else {
            return -1;
        }
    }

    public void vuforiaStop() {
        vuforia.close();
    }

}
