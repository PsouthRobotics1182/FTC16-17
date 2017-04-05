package org.firstinspires.ftc.team408.AustinsOld2017Ops;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

@TeleOp(name = "TeleOp_408", group = "TeleOp")
@Disabled
public class TeleOp_408 extends OpMode
{

    //drive motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor popperMotor;
    DcMotor lift, lift2;

    //Servos
    Servo buttonPush;
    Servo ball;

    ColorSensor color;
    LightSensor light;
    ModernRoboticsI2cRangeSensor rangeSensor;

    //drive values
    double left;
    double right;

    double liftPower;
    double popper;
    double buttonPusher;
    double AllTheWayUp = -5500;

    Boolean speed = true;
    Boolean ballCatch = true;
    Boolean breaks = false;

    int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution;

    @Override
    public void init()
    {

        //andymark motor specs
        int ticksPerRevolutionAndy = 1120;
        int maxRPMAndy = 129;
        int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;

        //Mapping physical motors to variable

        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        popperMotor = hardwareMap.dcMotor.get("popper");
        buttonPush = hardwareMap.servo.get("buttonPusher");
        light = hardwareMap.lightSensor.get("light sensor");
        ball = hardwareMap.servo.get("ball");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);

        lift = hardwareMap.dcMotor.get("Lift");
        lift2 = hardwareMap.dcMotor.get("Lift2");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        popperMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        light.enableLed(true);
    }

    @Override
    public void loop()
    {
        //Sets the brakes to on or off
        if (breaks == true)
        {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (breaks == false)
        {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //User control for the brakes
        if (gamepad1.x)
            breaks = !breaks;

        //User control for the drive train
        right = -gamepad1.left_stick_y;
        left = gamepad1.right_stick_y;

        //clipping values so they dont exceed 100% on motors
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);


        if (gamepad2.y)
        {
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (gamepad2.x)
        {
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //Controls the shooting mechanism
        if (gamepad2.b)
        {
            popper = 1;
            buttonPush.setPosition(0.0);
        }
        if (!gamepad2.b)
            popper = 0;


        //Controls the button pusher, putting it either left or right
        if (gamepad1.a)
            buttonPusher = 1;
        if (gamepad1.b)
            buttonPusher = 0.0;

        //User control for the speed of the robot
        if (gamepad1.dpad_up)
            speed = true;
        if (gamepad1.dpad_down)
            speed = false;

        //User control for the ball capper
        if (gamepad2.right_trigger >= 0.2)
        {
            liftPower = -(0.5) * (Math.sqrt(gamepad2.right_trigger));
            ballCatch = false;
        }

        if (gamepad2.left_trigger >= 0.2)
        {
            liftPower = (0.5) * (Math.sqrt(gamepad2.left_trigger));
            ballCatch = false;
        }

        if ( (gamepad2.left_trigger <= 0.2) && (gamepad2.right_trigger <= 0.2))
            liftPower = 0;


        /*if (lift.getCurrentPosition() <= 0 && liftPower <= 0)
        {
            liftPower = 0;
        }*/

        /*if (lift.getCurrentPosition() >= AllTheWayUp && liftPower >= 0 ) //This should make it to where it can only go so high but it doesn't seem to work 2-14-17
        {
            liftPower = 0;
        }*/

        //assigns the drive motors their variable speeds
        if (speed == true)
        {
            leftMotor.setPower(left);
            rightMotor.setPower(-right);
        }

        if (speed == false)
        {
            leftMotor.setPower(left * 0.25);
            rightMotor.setPower(-right * 0.25);
        }

        //Assigns the ball launcher and the button pusher their position or speed
        popperMotor.setPower(popper);
        buttonPush.setPosition(buttonPusher);


        //Assigns the ball capper its speed
        if (gamepad2.right_bumper)
        {
            lift.setPower(0.5 * -liftPower);
            lift2.setPower(0.5 * liftPower);
        }
        if (gamepad2.left_bumper)
        {
            lift.setPower(0.25 * -liftPower);
            lift2.setPower(0.25 * liftPower);
        }
        if ((gamepad2.right_bumper == false) && (gamepad2.left_bumper == false))
        {
            lift.setPower(-liftPower);
            lift2.setPower(liftPower);
        }

        //Assigns the ball holder, lift release mechanism its position
        if (ballCatch == true)
            ball.setPosition(0.6);
        if (ballCatch == false)
            ball.setPosition(0.0);

        //Returns telemetry data to the driver phone
        telemetry.addData("Left Motor power: ", left);
        telemetry.addData("Right Motor power: ", right);
        telemetry.addData("Popper Motor power: ", popper);
        telemetry.addData("Left Motor Position:", leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position:", rightMotor.getCurrentPosition());
        telemetry.addData("Light Sensor:", light.getRawLightDetected());
        telemetry.addData("Lift 1: ", lift.getCurrentPosition());
        telemetry.addData("Lift 2: ", lift2.getCurrentPosition());
        telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
        telemetry.addData("Lift Power: ", liftPower);


    }
}