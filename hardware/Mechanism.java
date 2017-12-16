package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Mechanism is an interface for all mechanisms on a robot. It contains methods and/or instance
 * variables common to all mechanisms.
 *
 * All robot mechanisms, including the main hardware map, should implement this interface.
 */
public interface Mechanism {

    /**
     * Initializes hardware on the robot. Gets and stores references to the robot configuration and
     * sets motors and servos to their starting positions.
     * @param hwMap     robot's hardware map
     */
    void init(HardwareMap hwMap);

}
