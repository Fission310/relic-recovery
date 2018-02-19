package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Acquirer is the class that is used to define all of the hardware for a robot's acquirer.
 * Acquirer must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the acquirer.
 */
public class Acquirer extends Mechanism {

    /* CONSTANTS */
    private static final double INTAKE_L_INIT = 0.1;
    private static final double INTAKE_L_ACT = 0.38;
    private static final double INTAKE_R_INIT = 0.5;
    private static final double INTAKE_R_ACT = 0.85;

    /* Hardware members */
    private DcMotor intakeL;
    private DcMotor intakeR;

    private Servo intakeLServo;
    private Servo intakeRServo;

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
        intakeLServo = hwMap.servo.get(RCConfig.INTAKE_L_SERVO);
        intakeRServo = hwMap.servo.get(RCConfig.INTAKE_R_SERVO);

        // Retrieve motor from hardware map and assign to instance vars
        intakeL = hwMap.dcMotor.get(RCConfig.INTAKE_L_MOTOR);
        intakeR = hwMap.dcMotor.get(RCConfig.INTAKE_R_MOTOR);

        // Set braking behavior
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

    /**
     * Set the acquirer servos to the activated position (to effectively acquire glyphs).
     */
    public void activate() {
        intakeLServo.setPosition(INTAKE_L_ACT);
        intakeRServo.setPosition(INTAKE_R_ACT);
    }

    /**
     * Set the acquirer servos to the deactivated position (to score glyphs.
     */
    public void deactivate() {
        intakeLServo.setPosition(INTAKE_L_INIT);
        intakeRServo.setPosition(INTAKE_R_INIT);
    }

    /**
     * Sets power for intake motor.
     */
    public void setIntakePower(double power) {
        activate();
        intakeL.setPower(power);
        intakeR.setPower(power);
    }

}
