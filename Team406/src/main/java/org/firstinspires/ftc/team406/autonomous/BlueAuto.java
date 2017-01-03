package org.firstinspires.ftc.team406.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team406.lib.AutoMethods;
import org.firstinspires.ftc.team406.lib.DriveSystem;

/**
 * Created by Robotics on 12/8/2016.
 */
@Autonomous
public class BlueAuto extends DriveSystem {
    @Override
    public void runOpMode() throws InterruptedException  {
        configure();

        waitForStart();

        drive(1120, 0.7);
        driveL(0.2);
        drive(80, 0.3);

        followLineRight(0.1);


    }
}
