package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Acquirer is the class that is used to define all of the hardware for a robot's acquirer.
 * Acquirer must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the acquirer. Assumes a clamping acquirer.
 */
public class LegacyAcquirer extends Mechanism {

    /* CONSTANTS */
    /**
     * Amount to change servo position every cycle
     */
    private static final double INCREMENT = 0.02;

    // Maximum rotational position
    private static final double MAX_POS = 1;
    // Minimum rotational position
    private static final double MIN_POS = 0.10; //0.3

    /* Hardware members */
    private DcMotor slides;

    private Servo left;
    private Servo right;


    /**
     * Default constructor for Acquirer.
     */
    public LegacyAcquirer(){

    }
    /**
     * Overloaded constructor for Acquirer. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public LegacyAcquirer(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes acquirer hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars
        left = hwMap.servo.get("left");
        right = hwMap.servo.get("right");

        // Retrieve motor from hardware map and assign to instance vars
        slides = hwMap.dcMotor.get("slides");

        // Set braking behavior
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial power
        slides.setPower(0);

        // Set initial position
        left.setPosition(MAX_POS);
        right.setPosition(MIN_POS);
    }

    /**
     * Accessor method for slides motor.
     * @return      slides motor
     */
    public DcMotor getSlides() {
        return slides;
    }

    /**
     * Clamps a glyph by setting the acquirer servos to a preset value.
     */
    public void clamp() {
        if (opMode.opModeIsActive()) {
            left.setPosition(MAX_POS / 3);
            right.setPosition((1 - MIN_POS) * 2 / 3);
        }
    }

    /**
     * Relases a glyph by setting the acquirer servos to a preset value.
     */
    public void release(LinearOpMode opMode) {
        if (opMode.opModeIsActive()) {
            left.setPosition(MAX_POS);
            right.setPosition(MIN_POS);
        }
    }

    /**
     * Increments the position of the acquiring servos in the direction specified.
     * @param is_clamp  <code>true</code> if the intention is to clamp, <code>false</code> otherwise
     */
    public void clampIncrement(boolean is_clamp) {

        // Positive or negative according to parameter
        int sign = is_clamp ? 1 : -1;

        // Increment servo position
        left.setPosition(left.getPosition() + sign * INCREMENT);
        right.setPosition(right.getPosition() - sign * INCREMENT);

        // If absolute servo position is past the max, set it back to the max or min
        if (left.getPosition() >= MAX_POS ) {
            left.setPosition(MAX_POS);
        } else if (right.getPosition() <= MIN_POS) {
            right.setPosition(MIN_POS);
        }
    }

    /**
     * Increments the position of the acquiring servos by the amount specified.
     * @param increment      amount to increment servo position
     */
    public void clampIncrement(double increment) {

        // Increment servo position
        left.setPosition(left.getPosition() + increment);
        right.setPosition(right.getPosition() - increment);

        // If absolute servo position is past the max, set it back to the max or min
        if (left.getPosition() >= MAX_POS ) {
            left.setPosition(MAX_POS);
        } else if (right.getPosition() <= MIN_POS) {
            right.setPosition(MIN_POS);
        }
    }

}
