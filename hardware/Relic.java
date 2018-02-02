package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Relic is the class that is used to define all of the hardware for a robot's relic mechanism.
 * Relic must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the relic.
 */
public class Relic extends Mechanism {

    /* CONSTANTS */
    public static final double MAX_TURN_POS = 1;
    public static final double MIN_TURN_POS = 0.45;
    public static final double INSIDE_TURN_POS = 0;
    public static final double MAX_CLAMP_POS = 1;
    public static final double MIN_CLAMP_POS = 0.5;

    /* Hardware members */
    private DcMotor slides;
    private Servo turn;
    private Servo clamp;

    /**
     * Default constructor for Relic.
     */
    public Relic(){ }

    /**
     * Overloaded constructor for Relic. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Relic(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        // Retrieve motor from hardware map and assign to instance vars
        slides = hwMap.dcMotor.get("slides");
        turn = hwMap.servo.get("turn");
        clamp = hwMap.servo.get("clamp");

        // Set braking behavior
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial power
        slides.setPower(0);

        // Set initial position
        turnDown();
        clamp();
    }

    /**
     * Sets power for slides motor.
     */
    public void setSlidesPower(double power) {
        slides.setPower(power);
    }

    /**
     * Sets the clamp servo to the clamped position.
     */
    public void clamp() {
        clamp.setPosition(MIN_CLAMP_POS);
    }

    /**
     * Sets the clamp servo to the maximum unclamped position.
     */
    public void unclamp() {
        clamp.setPosition(MAX_CLAMP_POS);
    }

    /**
     * Sets the turn servo to the down position.
     */
    public void turnDown() {
        turn.setPosition(MIN_TURN_POS);
    }

    /**
     * Sets the turn servo to the up position.
     */
    public void turnUp() {
        turn.setPosition(MAX_TURN_POS);
    }

    /**
     * Sets the turn servo to the inside position.
     */
    public void turnInside() {
        turn.setPosition(INSIDE_TURN_POS);
    }
}
