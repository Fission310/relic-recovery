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
    private static final double TURN_INSIDE_STATE = 1;
    private static final double TURN_NEUTRAL_STATE = 0.70;
    private static final double TURN_ACQ_STATE = 0;
    private static final double CLAMP_REL_STATE = 1;
    private static final double CLAMP_GET_STATE = 0.5;

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
        slides = hwMap.dcMotor.get(RCConfig.SLIDES);
        turn = hwMap.servo.get(RCConfig.TURN);
        clamp = hwMap.servo.get(RCConfig.CLAMP);

        // Set braking behavior
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial power
        slides.setPower(0);

        // Set initial position
        turnAcq();
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
        clamp.setPosition(CLAMP_GET_STATE);
    }

    /**
     * Sets the clamp servo to the maximum unclamped position.
     */
    public void unclamp() {
        clamp.setPosition(CLAMP_REL_STATE);
    }

    /**
     * Sets the turn servo to the acquiring position.
     */
    public void turnAcq() {
        turn.setPosition(TURN_ACQ_STATE);
    }

    /**
     * Sets the turn servo to the neutral position.
     */
    public void turnNeutral() {
        turn.setPosition(TURN_NEUTRAL_STATE);
    }

    /**
     * Sets the turn servo to the inside position.
     */
    public void turnInside() {
        turn.setPosition(TURN_INSIDE_STATE);
    }
}
