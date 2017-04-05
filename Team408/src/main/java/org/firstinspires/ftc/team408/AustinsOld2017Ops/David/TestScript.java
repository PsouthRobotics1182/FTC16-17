package org.firstinspires.ftc.team408.AustinsOld2017Ops.David;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics on 1/10/2017.
 **/

public class TestScript extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
   @Override
   public void runOpMode() throws InterruptedException {
        // Initialize motors.
        leftMotor = hardwareMap.dcMotor.get("LeftM");
        rightMotor = hardwareMap.dcMotor.get("RightM");
        // Wait for start of OpMode.
        waitForStart();
        // Run the motors at 1/10th power for 3 seconds.
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        telemetry.addData("TestValue","");
        sleep(3000);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
