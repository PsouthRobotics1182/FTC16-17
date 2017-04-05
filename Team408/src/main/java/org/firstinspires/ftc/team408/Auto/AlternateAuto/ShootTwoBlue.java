package org.firstinspires.ftc.team408.Auto.AlternateAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team408.Auto.AutoLib;

/**
 * Created by Robotics on 3/28/2017.
 */

@Autonomous(name = "Shoot Two Blue", group = "The Alternates")
public class ShootTwoBlue extends AutoLib {

    final static double FULL_POWER = 1; //Drive speeds
    final static double HALF_POWER = 0.5;
    final static double QUARTER_POWER = 0.25;

    @Override
    public void runOpMode() throws InterruptedException
    {
        config();
        waitForStart();

        sleep(20000);
        driveStraightFor(400, FULL_POWER);
        drivePower(0);
        sleep(1000);
        shootBalls();
        sleep(1000);

        rotateRight(0.72 * Math.PI / 2.5, QUARTER_POWER);
        driveStraightFor(1250, FULL_POWER);

        return;
    }
}
