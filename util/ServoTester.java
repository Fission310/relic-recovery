package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Util: Servo Test", group="Util")
public class ServoTester extends LinearOpMode {

    private static final String CONFIG_NAME = "turn";

    private Servo testServo;

    @Override
    public void runOpMode() {
        testServo = hardwareMap.servo.get(CONFIG_NAME);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Servo: ", testServo.getPosition());
            telemetry.update();
        }
    }
}
