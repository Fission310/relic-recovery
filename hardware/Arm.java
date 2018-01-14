package org.firstinspires.ftc.teamcode.hardware;


import android.graphics.Color;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Arm is the class that is used to define all of the hardware for a robot's arm.
 * Arm must be instantiated, then initialized using <code>init()</code> before being used.
 */
public class Arm extends Mechanism {

    /* CONSTANTS */
    /**
     * Hue threshold for blue jewel.
     */
    public static final double BLUE = 40;
    /**
     * Hue threshold for red jewel.
     */
    public static final double RED = 25;

    // Hue scale factor
    private static final double SCALE_FACTOR = 255 / 800;

    /* Hardware members */
    private ColorSensor sensorColor;
    private Servo arm;
    private JewelDetector jewelDetector;


    /**
     * Default constructor for Arm.
     */
    public Arm(){

    }
    /**
     * Overloaded constructor for Arm. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Arm(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes arm hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve color sensor from hardware map
        // sensorColor = hwMap.get(ColorSensor.class, "sensor_color");
        // sensorColor = hwMap.colorSensor.get("sensor_color");

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

        // Retrieve arm from hardware map and set to initial position
        arm = hwMap.servo.get("arm");
        arm.setPosition(1);
    }

    /**
     * Stops arm hardware.
     */
    public void stop() {
        jewelDetector.disable();
    }

    /**
     * Accessor method for arm servo.
     * @return      arm servo
     */
    public Servo getArm() {
        return arm;
    }

    /**
     * Gets calculated hue readings from the color sensor.
     * @return      array of hue values
     */
    public float[] getHSVValues() {
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues;
    }

    /**
     * Get the order of Jewels using DogeCV.
     * @return      BLUERED, REDBLUE, or UNDECIDED based on jewel order
     */
    public JewelDetector.JewelOrder getJewelOrder() {
        return jewelDetector.getCurrentOrder();
    }

}