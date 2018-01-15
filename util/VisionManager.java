package org.firstinspires.ftc.teamcode.util;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VisionManager {

    private JewelDetector jewelDetector;

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

}
