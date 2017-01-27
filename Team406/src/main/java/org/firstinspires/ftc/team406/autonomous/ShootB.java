package org.firstinspires.ftc.team406.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team406.lib.AutoMethods;

/**
 * Created by Robotics on 1/12/2017.
 */
@Autonomous(name="Shooting")
public class ShootB extends AutoMethods {
    public void runOpMode() throws InterruptedException{

        configure();
        autoSetup();

        waitForStart();

        //drives right distance to shoot into goal
        drive(500, 0.7);

        shoot();

        //moves ball from lift to shooter
        liftMotor.setPower(0.3);
        sleep(2000);
        liftMotor.setPower(0);

        shoot();

        drive(2700, 0.7);

    }
}
