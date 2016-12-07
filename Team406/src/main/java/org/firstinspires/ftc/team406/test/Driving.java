package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.DriveSystem;

/**
 * Created by dzogh_000 on 12/4/2016.
 */
@Autonomous
public class Driving extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveSystem robot = new DriveSystem();

        waitForStart();

        robot.drive(700, 0.5);

        sleep(1000);

        robot.pivotRight(Math.PI/2, 0.2);

        sleep(1000);

        robot.drive(700, 0.5);

        sleep(1000);

        robot.pivotRight(Math.PI/2, 0.2);

        sleep(1000);

        robot.drive(700, 0.5);

        sleep(1000);

        robot.pivotRight(Math.PI/2, 0.2);

        sleep(1000);

        robot.drive(700, 0.5);

        sleep(1000);

        robot.pivotRight(Math.PI/2, 0.2);

    }
}
