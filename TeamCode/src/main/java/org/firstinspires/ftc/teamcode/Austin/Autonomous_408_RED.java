package org.firstinspires.ftc.teamcode.Austin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 11/14/2016.
 */

@Autonomous(name = "Autonomous-408-RED", group = "LinearOpMode")
    public class Autonomous_408_RED extends LinearOpMode
    {
        DcMotor leftMotor;
        DcMotor rightMotor;
        DcMotor popper;
        DcMotor elevator;

        //Servos
        Servo pusher;
        Servo pusher2;

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

            pusher = hardwareMap.servo.get("pusher");
            pusher2 = hardwareMap.servo.get("pusher2");

            //set the motors run charictaristics
            //set direction
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            popper.setDirection(DcMotor.Direction.FORWARD);
            // or use runusingencoder to run by speed


            waitForStart();

            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            sleep(625);

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
            sleep(1000);

            rightMotor.setPower(1.0);
            leftMotor.setPower(-1.0);
            sleep(500);

            rightMotor.setPower(-1.0);
            leftMotor.setPower(1.0);
            sleep(500);

            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            sleep(500);


            leftMotor.setPower(0);
            rightMotor.setPower(0);




           // popper.setPower(1.0);
           // wait (250);

        }
    }



