package org.firstinspires.ftc.teamcode.Jeff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


//@TeleOp(name = "TeleOp_408", group = "TeleOp")
public class TeleOp_408Jeff extends OpMode
{

    //drive motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor popperMotor;
    DcMotor hopperControl;
    DcMotor elevatorControl;
    //Servos
    Servo pusher;
    Servo pusher2;
    Servo ballpush;

    //drive values
    double left;
    double right;
    double buttonpush;
    double hopper;
    double elevator;
    double popper;
    double baller;
    double buttonPush2;
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
        ballpush = hardwareMap.servo.get("baller");
        pusher2 = hardwareMap.servo.get("pusher2");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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


        if (gamepad1.a)
            buttonpush = 0.8;
        if (gamepad1.b)
            buttonpush = 0.0;

        if (gamepad1.x)
            buttonPush2 = 0.8;
        if (gamepad1.y)
            buttonPush2 = 0.0;

        if (gamepad2.right_bumper)
            popper = 1.0;
        if ( (gamepad2.right_bumper == false) && (gamepad1.right_trigger <= 0.2))
            popper = 0;
        if (gamepad2.right_trigger >= 0.2)
            popper = -1.0;

        if (gamepad2.dpad_up)
            elevator =  1.0;
        if (gamepad2.dpad_down)
            elevator = -1.0;
        if ( (gamepad2.dpad_up == false) && (gamepad1.dpad_down == false))
            elevator = 0;

        elevator = Range.clip(elevator, -1, 1);

        if (gamepad1.left_bumper)
            hopper = 1;
        if (gamepad1.left_trigger >= 0.2)
            hopper = -1.0;
        if ( (gamepad1.left_bumper == false) && (gamepad1.left_trigger <= 0.2))
            hopper = 0;

        if (gamepad2.dpad_left)
            baller = 0.0;
        if (gamepad2.dpad_right)
            baller = 1.0;
        if (gamepad2.dpad_right == false && gamepad2.dpad_left == false)
            baller = 0.5;



        //assign motors their values
        leftMotor.setPower(left);
        rightMotor.setPower(right);
        hopperControl.setPower(hopper);
        elevatorControl.setPower(elevator);
        pusher.setPosition(buttonpush);
        popperMotor.setPower(popper);
        ballpush.setPosition(baller);
        pusher.setPosition(buttonPush2);



        //telemetry
        telemetry.addData("Left Motor power: ", left);
        telemetry.addData("Right Motor power: ", right);
        telemetry.addData("Hopper Motor power: ", hopper);
        telemetry.addData("Elevator Motor power: ", elevator);
        telemetry.addData("Popper Motor power: ", popper);
        telemetry.addData("Pusher Servo Position: ", buttonpush);
        telemetry.addData("Ball Push Servo Position: ", baller);
        telemetry.addData("Pusher2 Servo Position: ", buttonPush2);

    }
}