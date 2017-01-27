package org.firstinspires.ftc.team406.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team406.lib.AutoMethods;
import org.firstinspires.ftc.team406.lib.DriveSystem;

/**
 * Created by Robotics on 1/9/2017.
 */
@Autonomous(name="DriveD test")
public class Tru extends AutoMethods {

    public void runOpMode() throws InterruptedException {
        configure();
        autoSetup();

        waitForStart();

        driveLF(1);
        driveLC(0.3);

        while(opModeIsActive() && !lineThereFront()){
            pivotRight(0.07);
        }
        stopMotors();
    }
}
