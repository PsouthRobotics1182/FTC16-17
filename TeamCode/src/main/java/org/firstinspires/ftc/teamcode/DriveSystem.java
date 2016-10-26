package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/*
 * Created by dzogh_000 on 9/17/2016.
 */
public class DriveSystem extends LinearOpMode {
    //Implementing nessasary methods
    @Override
    public void runOpMode(){}

    DcMotor leftMotor;
    DcMotor rightMotor;

    DcMotor[] motors = new DcMotor[2];

    //variables to run motors using encoders
    int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution;

    DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
    DcMotorSimple.Direction rightDirection, leftDirection;
    public DriveSystem(){

    }
    public void configureMotors(DcMotorSimple.Direction leftDirection, DcMotorSimple.Direction rightDirection){
        this.leftDirection = leftDirection;
        this.rightDirection = rightDirection;

        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");


        motors[0] = leftMotor;
        motors[1] = rightMotor;

        for (DcMotor motor : motors){
            motor.setMode(runMode);
            motor.setMaxSpeed(maxTicksPerSecond);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        setDirection("FORWARD");

    }
    public void drive(double power, int duration, String direction) throws InterruptedException{
        //todo
    }



    private void setDirection(String direction){
        // // TODO: 9/17/2016 find proper motor directions
        if (direction.equals("FORWARD")){
            leftMotor.setDirection(leftDirection);
            rightMotor.setDirection(rightDirection);
        }
        if (direction.equals("REVERSE")){
            leftMotor.setDirection(rightDirection);
            rightMotor.setDirection(leftDirection);
        }
        if (direction.equals("PIVOT_RIGHT")){
            leftMotor.setDirection(leftDirection);
            rightMotor.setDirection(leftDirection);
        }
        if (direction.equals("PIVOT_LEFT")){
            leftMotor.setDirection(rightDirection);
            rightMotor.setDirection(rightDirection);
        }
    }
}
