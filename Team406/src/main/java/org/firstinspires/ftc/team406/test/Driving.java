package org.firstinspires.ftc.team406.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team406.lib.DriveSystem;

/**
 * Created by dzogh_000 on 12/4/2016.
 */
//@Autonomous
public class Driving extends DriveSystem {
    @Override
    public void runOpMode() throws InterruptedException {
        configure();

        waitForStart();

        driveD(200, 0.1);


    }
}
