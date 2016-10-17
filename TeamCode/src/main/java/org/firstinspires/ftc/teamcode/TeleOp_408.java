package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.teamcode.OldOpmodes.*;


@TeleOp(name = "TeleOp_408", group = "TeleOp")
public class TeleOp_408 extends OpMode {

    //drive motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor hopperControl;

    //drive values
    double left;
    double right;
    double hopper;

    @Override
    public void init() {

        //Mapping physical motors to variable
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        hopperControl = hardwareMap.dcMotor.get("hopper");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        hopperControl.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        //clipping values so they dont exceed 100% on motors
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        //scale drive values for easier controlling
        left = Drive.expo(left);
        right = Drive.expo(right);

        if (gamepad1.dpad_up)
            hopper = 1.0;
        if (gamepad1.dpad_down)
            hopper = -1.0;
        else
            hopper = 0;


        leftMotor.setPower(left);
        rightMotor.setPower(right);
        hopperControl.setPower(hopper);

        //telemetry
        telemetry.addData("Left Motor power: ", left);
        telemetry.addData("Right Motor power: ", right);
        telemetry.addData("Hopper Motor power: ", hopper);
    }
}