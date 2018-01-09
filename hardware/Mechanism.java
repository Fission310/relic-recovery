package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Mechanism is an abstract class for all mechanisms on a robot. It contains methods and/or instance
 * variables common to all mechanisms.
 *
 * All robot mechanisms, including the main hardware map, should extend this abstract class.
 */
public abstract class Mechanism {

    /**
     * OpMode context for a Mechanism class.
     */
    protected LinearOpMode opMode;

    /**
     * Initializes hardware on the robot. Gets and stores references to the robot configuration and
     * sets motors and servos to their starting positions.
     * @param hwMap     robot's hardware map
     */
    public abstract void init(HardwareMap hwMap);

    /**
     * Stops hardware on the robot.
     */
    public abstract void stop();

    /**
     * Outputs the specified parameters to telemetry. Can only output one line at a time.
     * @param tag       description of outputted value
     * @param o         value to output
     */
    public void print(String tag, Object o) {
        opMode.telemetry.addData(tag, o);
        opMode.telemetry.update();
    }

}
