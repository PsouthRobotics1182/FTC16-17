package org.firstinspires.ftc.team406.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team406.lib.AutoMethods;
import org.firstinspires.ftc.team406.lib.DriveSystem;

/**
 * Created by Robotics on 12/8/2016.
 */
@Autonomous(name="automouse")
public class RedAuto extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException  {
        configure();
        autoSetup();

        waitForStart();

        driveLF(1);
        driveLC(0.3);

        //first button press
        while(opModeIsActive() && !lineThereFront()){
            pivotLeft(0.07);
            idle();
        }

        //here
        stopMotors();
        driveD(170,0.5);
        shoot();
        liftMotor.setPower(0.3);
        sleep(1500);
        liftMotor.setPower(0);
        shoot();
        telemetry.clearAll();
        telemetry.addData("PressingButton","");
        telemetry.update();
        pressButton("red");
        driveR(150, 200);

        //navigate to 2nd button
        pivotRight(Math.PI/2, 0.5);
        driveLF(1);
        driveLC(0.3);

        //second button press
        while(opModeIsActive() && !lineThereFront()){
            pivotRight(0.07);
            idle();
        }
        stopMotors();
        driveD(170, 0.5);
        telemetry.clearAll();
        telemetry.addData("PressingButton","");
        telemetry.update();
        pressButton("red");



    }
}
