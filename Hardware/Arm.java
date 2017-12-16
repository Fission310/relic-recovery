package org.firstinspires.ftc.teamcode.Hardware;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    public static final double BLUE = 115;
    public static final double RED = 85;
    public static final double SCALE_FACTOR = 255;

    private ColorSensor sensorColor;
    private Servo arm;

    public void init(HardwareMap hwMap) {
        sensorColor = hwMap.get(ColorSensor.class, "color_sensor");

        arm = hwMap.servo.get("arm");
        arm.setPosition(1);
    }

    public Servo getArm() {
        return arm;
    }

    public float[] getHSVValues() {
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues;
    }

}