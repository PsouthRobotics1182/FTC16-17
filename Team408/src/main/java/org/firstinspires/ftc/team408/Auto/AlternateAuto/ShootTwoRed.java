package org.firstinspires.ftc.team408.Auto.AlternateAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team408.Auto.AutoLib;

/**
 * Created by Robotics on 3/28/2017.
 */

@Autonomous(name = "Shoot Two Red", group = "The Alternates")
public class ShootTwoRed extends AutoLib {

    final static double FULL_POWER = 1; //Drive speeds
    final static double HALF_POWER = 0.5;
    final static double QUARTER_POWER = 0.25;

    @Override
    public void runOpMode() throws InterruptedException
    {
        config();
        waitForStart();

        sleep(10000);
        driveStraightFor(400, FULL_POWER);
        drivePower(0);
        sleep(1000);
        shootBalls();
        rotateLeft(0.45 * Math.PI / 2.5, QUARTER_POWER);
        driveStraightFor(1200, FULL_POWER);

        return;

    }
}

