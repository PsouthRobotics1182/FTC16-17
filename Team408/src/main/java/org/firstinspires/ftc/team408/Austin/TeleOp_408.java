package org.firstinspires.ftc.team408.Austin;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;



@TeleOp(name = "TeleOp_408", group = "TeleOp")
public class TeleOp_408 extends OpMode
{

    //drive motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor popperMotor;
    DcMotor lift;

    //Servos
    Servo buttonPush;
    Servo ball;


    ColorSensor color;
    LightSensor light;
    ModernRoboticsI2cRangeSensor rangeSensor;


    //drive values
    double left;
    double right;
    double UP = -1.0;
    double DOWN = 1.0;

    double liftPower;
    double popper;
    double buttonPusher;

    Boolean speed = true;
    Boolean ballCatch = true;



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
        right = -gamepad1.left_stick_y;
        left = gamepad1.right_stick_y;

        //clipping values so they dont exceed 100% on motors
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        if (gamepad2.right_bumper)
            popper = 1;
        if (gamepad2.right_bumper == false)
            popper = 0;

        if (gamepad1.a)
            buttonPusher = 1;
        if (gamepad1.b)
            buttonPusher = 0.0;

        if (gamepad1.dpad_up)
            speed = true;
        if (gamepad1.dpad_down)
            speed = false;


        if (gamepad2.left_bumper)
        {
            liftPower = UP;
            ballCatch = false;
        }
        if (gamepad2.left_trigger >= 0.2)
            liftPower = DOWN;
        if ( (gamepad2.left_bumper == false) && (gamepad2.left_trigger <= 0.2))
            liftPower = 0;

        //assign motors their values

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


        popperMotor.setPower(popper);
        buttonPush.setPosition(buttonPusher);
        lift.setPower(liftPower);

        if (ballCatch == true)
            ball.setPosition(0.6);
        if (ballCatch == false)
            ball.setPosition(0.0);






        //telemetry
        telemetry.addData("Left Motor power: ", left);
        telemetry.addData("Right Motor power: ", right);
        telemetry.addData("Popper Motor power: ", popper);
        telemetry.addData("Left Motor Position:", leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position:", rightMotor.getCurrentPosition());
        telemetry.addData("Light Sensor:", light.getRawLightDetected());




    }
}