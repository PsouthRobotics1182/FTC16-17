package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Robotics on 11/16/2016.
 */
@Autonomous
public class AutonomousClass extends LinearOpMode{

    public void runOpMode() throws InterruptedException{
        DriveSystem drive = new DriveSystem(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        waitForStart();
        //pivots so it is 90 to photo
        drive.pivot(90);
        //drives so it is straight out fro picture
        drive.drive(0.8, 50);
        //pivots to face picture
        drive.pivot(90);
    }
}
