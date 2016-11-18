package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//class to show new cqabilties

//register the class so it shows in app
//@Autonomous(name = "Example LinearOpMode", group = "LinearOpMode")
//@Disabled //hides the class in the app menu
public class AutonomousTest extends LinearOpMode {
    //define what motors are going to be in use
    DcMotor leftMotor;
    DcMotor rightMotor;

    //variables to run motors using encoders
    int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution;

    
    public void runOpMode() throws InterruptedException {
        //maps to the hardware motor using name provided in phone
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        //set the motors run charictaristics
        //set direction
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        // or use runusingencoder to run by speed
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set max speed for motor if being ran using speed
        leftMotor.setMaxSpeed(maxTicksPerSecond);
        rightMotor.setMaxSpeed(maxTicksPerSecond);
        // can be brake or float
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //resets encoders when you press start
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        waitForStart();

        //encoder tick value to reach controlled by PID
        leftMotor.setTargetPosition(4000);
        rightMotor.setTargetPosition(4000);
        //set power to move to this tick value
        leftMotor.setPower(.3);
        rightMotor.setPower(.3);
        // to reset encoder and enable drive by speed rather than power
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //run the motors at 0.5 speed
        rightMotor.setPower(.5);
        leftMotor.setPower(.5);
        //stop the motors
        rightMotor.setPower(0);
        leftMotor.setPower(0);
       
    }


}