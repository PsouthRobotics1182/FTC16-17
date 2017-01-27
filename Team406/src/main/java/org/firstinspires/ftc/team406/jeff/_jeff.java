package org.firstinspires.ftc.team406.jeff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics on 12/12/2016.
 */
public class _jeff extends OpMode {

    DcMotor LeftMotor;
    DcMotor RightMotor;
    public void init(){
        LeftMotor = hardwareMap.dcMotor.get("leftM");
        RightMotor = hardwareMap.dcMotor.get("rightM");
    }
    public void loop (){
        double LeftMotorPower = (gamepad1.left_stick_y);//gets value from gamepad
        double RightMotorPower = (gamepad1.right_stick_y);
        if (gamepad1.a){
            LeftMotorPower = LeftMotorPower/2;
            RightMotorPower = RightMotorPower/2;
        }
        LeftMotor.setPower(LeftMotorPower);
        RightMotor.setPower(RightMotorPower);

    }
}
