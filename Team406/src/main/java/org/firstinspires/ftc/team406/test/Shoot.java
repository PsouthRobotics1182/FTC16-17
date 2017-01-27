package org.firstinspires.ftc.team406.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team406.lib.AutoMethods;

/**
 * Created by Robotics on 1/12/2017.
 */
@Autonomous(name="Red Shooting")
public class Shoot extends AutoMethods {
    public void runOpMode() throws InterruptedException{

        configure();
        autoSetup();

        waitForStart();

        drive(500, 0.7);

        shoot();

        liftMotor.setPower(0.3);
        sleep(2000);
        liftMotor.setPower(0);

        shoot();

        drive(2200, 0.7);

        pivotLeft(Math.PI * 2);

        drive(400, 0.7);


    }
}
