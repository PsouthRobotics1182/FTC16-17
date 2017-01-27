package org.firstinspires.ftc.team406.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team406.lib.AutoMethods;
import org.firstinspires.ftc.team406.lib.DriveSystem;

/**
 * Created by Robotics on 12/8/2016.
 */
@Autonomous(name="automoose v2")
public class BlueAuto extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException  {
        configure();
        autoSetup();

        waitForStart();
        //drives full speed to till the first sensors sees the line to save time
        driveLF(1);
        //drive slowly to the second line for precise positions
        driveLC(0.3);
        //turn right till the front sensor sees the line
        while(opModeIsActive() && !lineThereFront()){
            pivotRight(0.07);
            idle();
        }
        //TODO weired pause after stopMotors completes
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //drives till sonar reads 17cm which is perfect distance to read color
        driveD(170,0.3);

        shoot();
        //moves balll out of lift to shooter
        liftMotor.setPower(0.3);
        sleep(1500);
        liftMotor.setPower(0);

        shoot();

        pressButton("blue");

        //drives back so we have room to pivot and turn
        driveR(730, 200);

        pivotLeft(Math.PI);

        drive(200, 0.7);


        /*//pivots left to face the second line
        pivotLeft(Math.PI/2, 0.5);

        //drives full speed to till the first sensors sees the line to save time
        driveLF(1);
        //drives slowly to second sensor to keep precision
        driveLC(0.3);

        //turns right until the front sensor sees the line so we are perpindicular ro wall
        while(opModeIsActive() && !lineThereFront()){
            pivotRight(0.07);
            idle();
        }
        //TODO weired pause after stopMotors completes
        stopMotors();

        //drives till sonar reads 17cm which is perfect distance for the lightsensor
        driveD(170, 0.5);

        pressButton("blue");*/



    }
}
