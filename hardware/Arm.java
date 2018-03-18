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
    private static final double ARM_INIT_POS = 0;
    private static final double ARM_UP_POS = 0.2;
    private static final double ARM_DOWN_POS = 0.8;
    private static final double SWEEPER_LEFT_POS = 1;
    private static final double SWEEPER_NEUTRAL_POS = 0.55;
    private static final double SWEEPER_RIGHT_POS = 0;

    /* Hardware members */
    private Servo arm;
    private Servo sweeper;


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
        arm = hwMap.servo.get(RCConfig.ARM);
        sweeper = hwMap.servo.get(RCConfig.SWEEPER);
        armInit();
        sweeperNeutral();
    }

    /**
     * Set the arm to the init position.
     */
    public void armInit() {
        arm.setPosition(ARM_INIT_POS);
    }

    /**
     * Set the arm to the up position.
     */
    public void armUp() {
        arm.setPosition(ARM_UP_POS);
    }

    /**
     * Set the arm to the down position
     */
    public void armDown() {
        arm.setPosition(ARM_DOWN_POS);
    }

    /**
     * Set the sweeper to the left position.
     */
    public void sweeperLeft() {
        sweeper.setPosition(SWEEPER_LEFT_POS);
    }

    /**
     * Set the sweeper to the neutral position.
     */
    public void sweeperNeutral() {
        sweeper.setPosition(SWEEPER_NEUTRAL_POS);
    }

    /**
     * Set the sweeper to the right position.
     */
    public void sweeperRight() {
        sweeper.setPosition(SWEEPER_RIGHT_POS);
    }

}