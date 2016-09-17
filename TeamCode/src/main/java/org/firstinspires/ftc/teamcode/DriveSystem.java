package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by dzogh_000 on 9/17/2016.
 */
public class DriveSystem extends LinearOpMode {
    //Implementing nessasary methods
    @Override
    public void runOpMode(){}

    DcMotor leftMotor;
    DcMotor leftFMotor;
    DcMotor rightMotor;
    DcMotor rightFMotor;

    //variables to run motors using encoders
    int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution;
    List<DcMotor> motors = new ArrayList<>();

    int wheelcount;
    DcMotor.RunMode runMode;
    DcMotorSimple.Direction rightDirection, leftDirection;
    public DriveSystem(int wheelCount, DcMotorSimple.Direction leftDirection, DcMotorSimple.Direction rightDirection){
        this.wheelcount = wheelCount;
        this.leftDirection = leftDirection;
        this.rightDirection = leftDirection;
    }
    public void configureMotors(){
        if(wheelcount == 2){

            leftMotor = hardwareMap.dcMotor.get("leftM");
            rightMotor = hardwareMap.dcMotor.get("rightM");


            motors.add(leftMotor);
            motors.add(rightMotor);
            if (wheelcount == 4){
                motors.add(leftFMotor);
                motors.add(rightFMotor);
            }

            for (DcMotor motor : motors){
                motor.setMode(runMode);
                motor.setMaxSpeed(maxTicksPerSecond);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            setDirection("FORWARD");
        }

    }
    public void driveTime(double power, int duration, String direction) throws InterruptedException{
        for (DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setDirection(direction);
            motor.setPower(power);
            sleep(duration);
            motor.setPower(0);
        }
    }
    public void driveDistance(double power, int duration, String direction) {
        //// TODO: 9/17/2016  make this method for autoomous library
    }


    public void setDirection(String direction){
        // // TODO: 9/17/2016 find proper motor directions
        if (direction.equals("FORWARD")){
            leftMotor.setDirection(leftDirection);
            leftFMotor.setDirection(leftDirection);
            rightMotor.setDirection(rightDirection);
            rightMotor.setDirection(rightDirection);
        }
        if (direction.equals("REVERSE")){
            leftMotor.setDirection(rightDirection);
            leftFMotor.setDirection(rightDirection);
            rightMotor.setDirection(leftDirection);
            rightMotor.setDirection(leftDirection);
        }
        if (direction.equals("PIVOT_RIGHT")){
            leftMotor.setDirection(leftDirection);
            leftFMotor.setDirection(leftDirection);
            rightMotor.setDirection(leftDirection);
            rightMotor.setDirection(leftDirection);
        }
        if (direction.equals("PIVOT_LEFT")){
            leftMotor.setDirection(rightDirection);
            leftFMotor.setDirection(rightDirection);
            rightMotor.setDirection(rightDirection);
            rightMotor.setDirection(rightDirection);
        }
    }
}
