package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OldOpmodes.Drive;


//@TeleOp(name = "TestBot", group = "TeleOp")
public class TestBot extends OpMode {
    //drive motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    //drive values
    double left;
    double right;
    double a = 1;
    //scale for expo

    @Override
    public void init() {

        //Mapping physical motors to variable
        leftMotor = hardwareMap.dcMotor.get("left_Motor");
        rightMotor = hardwareMap.dcMotor.get("right_Motor");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        //getting drive values from controller
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        //clipping values so they dont exceed 100% on motors
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);
        //scale drive values for easier controlling
        left = Drive.expo(left);
        right = Drive.expo(right);


        if (gamepad2.dpad_down)
            a = 0.3;
        if (gamepad2.dpad_up)
            a = 1;

        //set drive values for
        left = left * a;
        right = right * a;
        leftMotor.setPower(left);
        rightMotor.setPower(right);

        //telemetry
        telemetry.addData("Left Motor power", left);
		telemetry.addData("Right Motor power", right);
        telemetry.addData("Speed", a);
    }


}