package org.firstinspires.ftc.team406.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team406.lib.DriveSystem;

/**
 * Created by dzogh_000 on 12/4/2016.
 */
@Autonomous
public class Driving extends DriveSystem {
    @Override
    public void runOpMode() throws InterruptedException {
        configure();

        waitForStart();

        drive(700, 0.5);

        sleep(1000);

        pivotLeft(Math.PI/2, 0.2);

        sleep(1000);

        drive(700, 0.5);

        sleep(1000);

        pivotLeft(Math.PI/2, 0.2);

        sleep(1000);

        drive(700, 0.5);

        sleep(1000);

        pivotLeft(Math.PI/2, 0.2);

        sleep(1000);

        drive(700, 0.5);

        sleep(1000);

        pivotLeft(Math.PI/2, 0.2);

        sleep(1000);

        drive(100, 0.5);

        sleep(1000);

        turnLeft(300, Math.PI/2, 0.2);

        sleep(1000);

        drive(100, 0.5);

        sleep(1000);

        turnLeft(300, Math.PI/2, 0.2);

        sleep(1000);

        drive(100, 0.5);

        sleep(1000);

        turnLeft(300, Math.PI/2, 0.2);

        sleep(1000);

        drive(100, 0.5);

        sleep(1000);

        turnLeft(300, Math.PI/2, 0.2);


    }
}
