package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//class to show new cqabilties

//register the class so it shows in app
@TeleOp(name = "ProTeleOp", group = "TeleOp")
//@Disabled //hides the class in the app menu
public class ProTeleOp extends OpMode {
    //define what motors are going to be in use
    DcMotor leftMotor;
    DcMotor rightMotor;

    //variables to run motors using encoders
    int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution;

    @Override
    public void init() {
        //maps to the hardware motor using name provided in phone
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        //set the motors run charictaristics
        //set direction
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        // or use runusingencoder to run by speed
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set max speed for motor if being ran using speed
        leftMotor.setMaxSpeed(maxTicksPerSecond);
        rightMotor.setMaxSpeed(maxTicksPerSecond);
        // can be brake or float
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop(){
        //resets encoders when you press start
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;

        leftPower = scale(leftPower);
        rightPower = scale(rightPower);

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

    }

    public double scale(double power){
        power = power * power * power;
        return power;
    }

}