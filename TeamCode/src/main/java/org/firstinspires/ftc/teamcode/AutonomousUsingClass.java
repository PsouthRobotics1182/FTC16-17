package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AutonomousUsingClass extends LinearOpMode {
    DriveSystem wheels = new DriveSystem();

    @Override
    public void runOpMode() throws InterruptedException{
        wheels.configureMotors(2, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);

        waitForStart();

        wheels.driveTime(1, 1000, "FORWARD");
    }
}
