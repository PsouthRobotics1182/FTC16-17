package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.teamcode.OldOpmodes.*;


@TeleOp(name = "TeleOp_408", group = "TeleOp")
public class TeleOp_408 extends OpMode
{

    //drive motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor popperMotor;
    DcMotor hopperControl;
    DcMotor elevatorControl;
    //Servos
    Servo pusher;

    //drive values
    double left;
    double right;
    double buttonpush;
    double hopper;
    double elevator;
    double popper;
   /* int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution; */

    @Override
    public void init()
    {

        //Mapping physical motors to variable
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        hopperControl= hardwareMap.dcMotor.get("hopper");
        elevatorControl = hardwareMap.dcMotor.get("elevator");
        popperMotor = hardwareMap.dcMotor.get("popper");

        pusher = hardwareMap.servo.get("pusher");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        popperMotor.setDirection(DcMotor.Direction.FORWARD);



    }

    @Override
    public void loop()
    {
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        //clipping values so they dont exceed 100% on motors
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        //scale drive values for easier controlling
        left = Drive.expo(left);
        right = Drive.expo(right);



        if (gamepad1.a)
            buttonpush = 1.0;
        if (gamepad1.b)
            buttonpush = -1.0;

        if (gamepad1.right_bumper)
            popper = 1.0;
        if (!(gamepad1.right_bumper == true))
            popper = 0.0;
        if (gamepad1.right_trigger >= 0.2)
            popper = -1.0;

        if (gamepad1.dpad_up)
            elevator =  1.0;
        if (gamepad1.dpad_down)
            elevator = -1.0;
        if ( (gamepad1.dpad_up == false) && (gamepad1.dpad_down == false))
            elevator = 0;

        if (gamepad1.left_bumper)
            hopper = 1;
        if (gamepad1.left_trigger >= 0.2)
            hopper = 1.0;
        if ( (gamepad1.left_bumper == false) && (gamepad1.left_trigger <= 0.2))
            hopper = 0;



        //assign motors their values
        leftMotor.setPower(left);
        rightMotor.setPower(right);
        hopperControl.setPower(hopper);
        elevatorControl.setPower(elevator);
        pusher.setPosition(buttonpush);

        //telemetry
        telemetry.addData("Left Motor power: ", left);
        telemetry.addData("Right Motor power: ", right);
        telemetry.addData("Hopper Motor power: ", hopper);
        telemetry.addData("Elevator Motor power: ", elevator);
        telemetry.addData("Pusher Servo Position: ", buttonpush);

    }
}