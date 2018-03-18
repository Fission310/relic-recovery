package org.firstinspires.ftc.teamcode.hardware.mecanum;


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
import org.firstinspires.ftc.teamcode.hardware.RCConfig;

/**
 * Drivetrain is the class that is used to define all of the hardware for a robot's drivetrain.
 * Drivetrain must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the drivetrain. <code>encoderInit()</code>
 * must be called before an autonomous action can be called.
 *
 * This class describes a mecanum drivetrain.
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
    public static final double     DRIVE_SPEED             = 0.4;

    // Constant adjusting value for encoder driving
    private static final double     PCONSTANT               = 0.1;

    /* Hardware members */
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

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
        leftFront = hwMap.dcMotor.get(RCConfig.LEFT_FRONT);
        leftBack = hwMap.dcMotor.get(RCConfig.LEFT_BACK);
        rightFront = hwMap.dcMotor.get(RCConfig.RIGHT_FRONT);
        rightBack = hwMap.dcMotor.get(RCConfig.RIGHT_BACK);

        // Set motor direction (AndyMark configuration)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

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
        imu = hwMap.get(BNO055IMU.class, RCConfig.IMU);
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
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Set drivetrain motor power based on input.
     *
     * @param x         x component of drive vector
     * @param y         y component of drive vector
     * @param turn      turn vector
     */
    public void drive(double x, double y, double turn) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double v1 = r * Math.cos(robotAngle) + turn;
        double v2 = r * Math.sin(robotAngle) - turn;
        double v3 = r * Math.sin(robotAngle) + turn;
        double v4 = r * Math.cos(robotAngle) - turn;

        leftFront.setPower(v3); // v2
        leftBack.setPower(v1); // v4
        rightBack.setPower(v2); // v3
        rightFront.setPower(v4); // v1
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
     * @param inches        number of inches to drive
     * @param timeoutS      amount of time before the move should stop
     */
    public void driveToPos(double inches, double timeoutS) {

        // Drivetrain adjustments
        inches = -inches;

        // Target position variables
        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightFrontTarget = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newLeftBackTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightBackTarget = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

            // Set power of drivetrain motors accounting for adjustment
            leftFront.setPower(Drivetrain.DRIVE_SPEED);
            rightFront.setPower(-Drivetrain.DRIVE_SPEED);
            leftBack.setPower(Drivetrain.DRIVE_SPEED);
            rightBack.setPower(-Drivetrain.DRIVE_SPEED);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %.2f :%.2f :%.2f :%.2f",
                    newLeftFrontTarget / COUNTS_PER_INCH,
                    newRightFrontTarget / COUNTS_PER_INCH,
                    newLeftBackTarget / COUNTS_PER_INCH,
                    newRightBackTarget / COUNTS_PER_INCH);
            double[] positions = getPositions();
            opMode.telemetry.addData("Path2", "Running at %.2f :%.2f :%.2f :%.2f",
                    positions[0],
                    positions[1],
                    positions[2],
                    positions[3]);
            opMode.telemetry.update();
        }

        // Stop all motion
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Turn to a specified angle relative to the robot's starting position using an IMU.
     *
     * Robot will stop moving if any of three conditions occur:
     * <li>
     *  <ol>Move gets to the desired angle</ol>
     *  <ol>Move runs out of time</ol>
     *  <ol>Driver stops the running OpMode</ol>
     * </li>
     *
     * @param targetAngle   number of degrees to turn
     * @param timeoutS      amount of time before the move should stop
     */
    public void turn(double targetAngle, double timeoutS) {

        // Normalize input
        //targetAngle = targetAngle % 360;

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() && Math.abs(getError(targetAngle)) > 1.5 && runtime.seconds() < timeoutS) {

            double velocity = getError(targetAngle) / 180 + 0.1; // this works
            //double velocity = Math.max(getError(targetAngle) / 180 * 2, 0.25); // to be tested why doesn't this work

            // Set motor power according to calculated angle to turn
            leftFront.setPower(velocity);
            rightFront.setPower(-velocity);
            leftBack.setPower(velocity);
            rightBack.setPower(-velocity);

            // Display heading for the driver
            opMode.telemetry.addData("Heading: ", "%.2f : %.2f", targetAngle, getHeading());
            opMode.telemetry.addData("Velocity: ", "%.2f", velocity);
            opMode.telemetry.update();
        }

        // Stop motor movement
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * Get heading of the robot relative to the initialized position.
     * @return      heading of the robot
     */
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // Get the difference in the target angle and the current heading
    private double getError(double targetAngle) {
        double heading = getHeading();
        if (targetAngle > heading) {
            if (targetAngle - heading > 180) {
                return 360 - Math.abs(targetAngle) - Math.abs(heading);
            } else {
                return targetAngle - heading;
            }
        } else {
            if (targetAngle - heading > 180) {
                return -(360 - Math.abs(targetAngle) - Math.abs(heading));
            } else {
                return heading - targetAngle;
            }
        }
    }

    /**
     * Get encoder positions of the drivetrain.
     * @return      double array with the encoder values of each drivetrain motor
     */
    public double[] getPositions() {
        double[] positions = new double[4];
        positions[0] = leftFront.getCurrentPosition() / COUNTS_PER_INCH;
        positions[1] = rightFront.getCurrentPosition() / COUNTS_PER_INCH;
        positions[2] = leftBack.getCurrentPosition() / COUNTS_PER_INCH;
        positions[3] = rightBack.getCurrentPosition() / COUNTS_PER_INCH;

        return positions;
    }
}
