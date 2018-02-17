package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Flipper is the class that is used to define all of the hardware for a robot's flipper.
 * Flipper must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the flipper/hopper.
 */
public class Flipper extends Mechanism {

    /* CONSTANTS */
    public static final double FLIP_L_ACQ_STATE =      0;
    public static final double FLIP_R_ACQ_STATE =      1 - FLIP_L_ACQ_STATE;
    public static final double FLIP_L_NEUTRAL_STATE =  0.2;
    public static final double FLIP_R_NEUTRAL_STATE =  1 - FLIP_L_NEUTRAL_STATE;
    public static final double FLIP_L_SCORE_STATE =    0.7;
    public static final double FLIP_R_SCORE_STATE =    1 - FLIP_L_SCORE_STATE;

    /* Hardware members */
    private DcMotor lift;
    private Servo flipL;
    private Servo flipR;

    /**
     * Default constructor for Flipper.
     */
    public Flipper(){

    }
    /**
     * Overloaded constructor for Flipper. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Flipper(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes acquirer hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars
        flipL = hwMap.servo.get(RCConfig.FLIP_L);
        flipR = hwMap.servo.get(RCConfig.FLIP_R);

        // Retrieve motor from hardware map and assign to instance vars
        lift = hwMap.dcMotor.get(RCConfig.LIFT);

        // Set braking behavior
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial power
        lift.setPower(0);

        flipL.setPosition(FLIP_L_ACQ_STATE);
        flipR.setPosition(FLIP_R_ACQ_STATE);
    }


    /**
     * Sets power for lift motor.
     */
    public void setLiftPower(double power) {
        lift.setPower(power);
    }

}
