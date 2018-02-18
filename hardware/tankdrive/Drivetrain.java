package org.firstinspires.ftc.teamcode.hardware.tankdrive;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

/**
 * Drivetrain is the class that is used to define all of the hardware for a robot's drivetrain.
 * Drivetrain must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the drivetrain. <code>encoderInit()</code>
 * must be called before an autonomous action can be called.
 *
 * This class describes a tankdrive drivetrain.
 */
public class Drivetrain extends Mechanism {

    /* CONSTANTS */
    /**
     * Ticks per revolution for a NeverRest 40.
     */
    private static final double     COUNTS_PER_MOTOR_REV    = 1120;
    /**
     * Drivetrain gear ratio (< 1.0 if geared up).
     */
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    /**
     * Diameter of wheel in inches.
     */
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    /**
     * Calculated ticks per inch.
     */
    private static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    /**
     * Drive speed when using encoders.
     */
    private static final double     DRIVE_SPEED             = 0.4;
    /**
     * Turn speed when using encoders.
     */
    private static final double     TURN_SPEED              = 0.3;

    // Constant adjusting value for encoder driving
    private static final double     PCONSTANT               = 0.1;

    /* Hardware members */
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private BNO055IMU imu;


    /**
     * Default constructor for Drivetrain.
     */
    public Drivetrain(){

    }
    /**
     * Overloaded constructor for Drivetrain. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Drivetrain(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes drivetrain hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {

        // Retrieve motors from hardware map and assign to instance vars
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");

        // Set motor direction (AndyMark configuration)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor brake behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Initialize IMU with parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /**
     * Initializes motors for encoder driving. Must be called before calling methods that use
     * encoders.
     */
    public void encoderInit() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Set drivetrain motor power based on input.
     *
     * @param left      power for left side of drivetrain (-1 to 1)
     * @param right     power for right side of drivetrain (-1 to 1)
     */
    public void drive(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightBack.setPower(right);
        rightFront.setPower(right);
    }

    /**
     * Drive to a relative position using encoders and an IMU.
     *
     * Robot will stop moving if any of three conditions occur:
     * <ul>
     *  <li>Move gets to the desired position</li>
     *  <li>Move runs out of time</li>
     *  <li>Driver stops the running OpMode</li>
     * </ul>
     *
     * @param speed         maximum power of drivetrain motors when driving
     * @param leftInches    number of inches to move on the left side
     * @param rightInches   number of inches to move on the right side
     * @param timeoutS      amount of time before the move should stop
     */
    public void driveToPos(double speed, double leftInches, double rightInches, double timeoutS) {

        // Target position variables
        int newLeftTarget;
        int newRightTarget;

        // Current heading angle of robot
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        leftFront.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                leftFront.isBusy() && rightFront.isBusy()) {

            // Get IMU angles
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Heading angle
            double gyroAngle = angles.firstAngle;

            // Adjustment factor for heading
            double p = (gyroAngle - currentAngle) * PCONSTANT;

            // Set power of drivetrain motors accounting for adjustment
            leftFront.setPower(Math.abs(speed) + p);
            rightFront.setPower(Math.abs(speed) - p);

                /*
                if (leftInches < 0) {
                    leftBack.setPower(-Math.abs(speed));
                } else {
                    leftBack.setPower(Math.abs(speed));
                }
                if (rightInches < 0) {
                    rightBack.setPower(-Math.abs(speed));
                } else {
                    rightBack.setPower(Math.abs(speed));
                }*/

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition());
            opMode.telemetry.addData("Heading: ", "%f", gyroAngle);
            opMode.telemetry.addData("AccX: ", "%f", imu.getAcceleration().xAccel);
            opMode.telemetry.addData("AccY: ", "%f", imu.getAcceleration().yAccel);
            opMode.telemetry.update();
        }

        // Stop all motion
        leftFront.setPower(0);
        rightFront.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Turn to a specified angle using an IMU.
     *
     * Robot will stop moving if any of three conditions occur:
     * <li>
     *  <ol>Move gets to the desired angle</ol>
     *  <ol>Move runs out of time</ol>
     *  <ol>Driver stops the running OpMode</ol>
     * </li>
     *
     * @param speed         maximum power of drivetrain motors when driving
     * @param angle         number of degrees to turn
     * @param timeoutS      amount of time before the move should stop
     */
    public void turn(double speed, double angle, double timeoutS) {
        // Get IMU angles
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Calculate angle to turn
        double angleToTurn = angle - angles.firstAngle;

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() && Math.abs(angleToTurn) > 0.5 && runtime.seconds() < timeoutS) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleToTurn = angle - angles.firstAngle;

            // Set motor power according to calculated angle to turn
            leftFront.setPower(-Math.signum(angleToTurn) * speed);
            rightFront.setPower(Math.signum(angleToTurn) * speed);
            leftBack.setPower(-Math.signum(angleToTurn) * speed);
            rightBack.setPower(Math.signum(angleToTurn) * speed);

            // Display heading for the driver
            opMode.telemetry.addData("Heading: ", "%f", angleToTurn);
            opMode.telemetry.update();
        }

        // Stop motor movement
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
