/*
package org.firstinspires.ftc.team406.OldOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

*/
/**
 * Created by Robotics on 11/14/2016.
 *//*


@Autonomous(name = "Autonomous-408", group = "LinearOpMode")
    public class Autonomous_408 extends LinearOpMode
    {
        DcMotor leftMotor;
        DcMotor rightMotor;
        DcMotor popper;
        DcMotor elevator;

        //variables to run motors using encoders
        int ticksPerRevolution = 1440;
        int maxRPM = 152;
        int maxTicksPerSecond = maxRPM * ticksPerRevolution;

        public void runOpMode() throws InterruptedException
        {
            //maps to the hardware motor using name provided in phone
            leftMotor = hardwareMap.dcMotor.get("leftM");
            rightMotor = hardwareMap.dcMotor.get("rightM");
            popper = hardwareMap.dcMotor.get("popper");
            elevator = hardwareMap.dcMotor.get("elevator");
            //set the motors run charictaristics
            //set direction
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            popper.setDirection(DcMotor.Direction.FORWARD);
            // or use runusingencoder to run by speed


            waitForStart();

            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            sleep(1500);

            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            sleep(750);

            popper.setPower(-1.0);
            sleep(1000);

            popper.setPower(0);
            elevator.setPower(1.0);
            sleep(3000);

            elevator.setPower(0);
            popper.setPower(-1.0);
            sleep(2000);

            popper.setPower(0);
            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            sleep(2000);

            leftMotor.setPower(0);
            rightMotor.setPower(0);




           // popper.setPower(1.0);
           // wait (250);

        }
    }



*/
