package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Acquirer {

    public static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    public static final double MAX_POS = 1.0;     // Maximum rotational position
    public static final double MIN_POS = 0.0;     // Minimum rotational position

    private DcMotor slides;

    private Servo left;
    private Servo right;

    public void init(HardwareMap hwMap) {
        left = hwMap.servo.get("bottomLeft");
        right = hwMap.servo.get("bottomRight");

        slides = hwMap.dcMotor.get("slides");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setPower(0);

        left.setPosition(MAX_POS);
        right.setPosition(MIN_POS);
    }

    public DcMotor getSlides() {
        return slides;
    }

    public void clamp(LinearOpMode opMode) {
        if (opMode.opModeIsActive()) {
            left.setPosition(MAX_POS / 3);
            right.setPosition((1 - MIN_POS) * 2 / 3);
        }
    }

    public void release(LinearOpMode opMode) {
        if (opMode.opModeIsActive()) {
            left.setPosition(MAX_POS);
            right.setPosition(MIN_POS);
        }
    }

    public void clampIncrement(boolean is_clamp) {
        int sign = is_clamp ? 1 : -1;

        left.setPosition(left.getPosition() + sign * INCREMENT);
        right.setPosition(right.getPosition() - sign * INCREMENT);

        if (left.getPosition() >= MAX_POS ) {
            left.setPosition(MAX_POS);
        } else if (right.getPosition() <= MIN_POS) {
            right.setPosition(MIN_POS);
        }
    }

    public void clampIncrement(double increment) {
        left.setPosition(left.getPosition() + increment);
        right.setPosition(right.getPosition() - increment);

        if (left.getPosition() >= MAX_POS ) {
            left.setPosition(MAX_POS);
        } else if (right.getPosition() <= MIN_POS) {
            right.setPosition(MIN_POS);
        }
    }

}
