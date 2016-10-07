package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutonomousUsingClass extends LinearOpMode {
    DriveSystem wheels;

    @Override
    public void runOpMode() throws InterruptedException{
        wheels = new DriveSystem();
        wheels.configureMotors(2, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);

        waitForStart();

        wheels.driveTime(1, 1000, "FORWARD");
    }
}
