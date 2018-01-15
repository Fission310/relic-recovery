package org.firstinspires.ftc.teamcode.hardware;


import android.graphics.Color;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

        // Retrieve arm from hardware map and set to initial position
        arm = hwMap.servo.get("arm");
        arm.setPosition(1);
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



}