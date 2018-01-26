package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Acquirer is the class that is used to define all of the hardware for a robot's drivetrain.
 * Acquirer must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the acquirer.
 */
public class Acquirer extends Mechanism {

    /* CONSTANTS */
    /**
     * Amount to change servo position every cycle
     */
    public static final double INCREMENT = 0.02;

    // Maximum rotational position
    private static final double MAX_POS = 1;
    // Minimum rotational position
    private static final double MIN_POS = 0.10; //0.3

    /* Hardware members */
    private DcMotor intake;


    /**
     * Default constructor for Acquirer.
     */
    public Acquirer(){

    }
    /**
     * Overloaded constructor for Acquirer. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Acquirer(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes acquirer hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars

        // Retrieve motor from hardware map and assign to instance vars
        intake = hwMap.dcMotor.get("intake");

        // Set braking behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial power
        intake.setPower(0);
    }

    /**
     * Sets power for intake motor.
     */
    public void setIntakePower(double power) {
        intake.setPower(power);
    }

}
