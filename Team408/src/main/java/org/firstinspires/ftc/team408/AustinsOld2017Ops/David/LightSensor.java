package org.firstinspires.ftc.team408.AustinsOld2017Ops.David;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics on 1/11/2017.
 */

public class LightSensor extends LinearOpMode
{
    DcMotor leftMotor;
    DcMotor rightMotor;
    double FULL_POWER = 1.0, HALF_POWER = 0.5, QUARTER_POWER = 0.25, NO_POWER = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        initMotors();
        turnRight(HALF_POWER, 3000);
    }
    // Initialize motors.
    public void initMotors() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        waitForStart();
    }
    // Stop motors.
    public void stopMotors()
    {
        leftMotor.setPower(NO_POWER);
        rightMotor.setPower(NO_POWER);
    }
    // Turn right.
    public void turnRight(double power, int num) throws InterruptedException
    {
        leftMotor.setPower(power);
        rightMotor.setPower(NO_POWER);
        sleep(num);
        stopMotors();
    }
    // Turn left.
    public void turnLeft(double power, int num) throws InterruptedException
    {
        leftMotor.setPower(NO_POWER);
        rightMotor.setPower(power);
        sleep(num);
        stopMotors();
    }
}
