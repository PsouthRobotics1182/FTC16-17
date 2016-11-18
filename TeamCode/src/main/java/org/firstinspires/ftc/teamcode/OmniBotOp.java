package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OldOpmodes.Drive;


//@TeleOp(name = "OmniBot", group = "TeleOp")
public class OmniBotOp extends OpMode {

    //drive motors
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightBackMotor;


    //Servos
    Servo pusher;

    //drive values
    double leftFront;
    double rightBack;
    double leftBack;
    double rightFront;

    double Horizontal;
    double Vertical;
    double Diagonal;

    double angle;


    @Override
    public void init() {

        //Mapping physical motors to variable
        leftFrontMotor = hardwareMap.dcMotor.get("leftFM");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFM");
        leftBackMotor = hardwareMap.dcMotor.get("leftBM");
        rightBackMotor = hardwareMap.dcMotor.get("rightBM");



        pusher = hardwareMap.servo.get("pusher");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {


        Horizontal = (gamepad1.left_stick_x);
        Vertical = (gamepad1.left_stick_y);
        Diagonal = (Vertical * Horizontal);


        //clipping values so they dont exceed 100% on motors
        Vertical = Range.clip(Vertical, -1, 1);
        Horizontal = Range.clip(Horizontal, -1, 1);
        Diagonal = Range.clip(Diagonal, -1, 1);

        angle = Math.atan(Vertical / Horizontal);



        if ( (angle >= ((3 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((5 * Math.PI)/ 16) * (180 / Math.PI)) ) //Diagonal Up Right
        {
            rightFront = 0;
            rightBack = Diagonal;
            leftFront = Diagonal;
            leftBack = 0;
        }

        if ( (angle >= ((11 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((13 * Math.PI)/ 16) * (180 / Math.PI)) ) //Diagonal Up Left
        {
            rightBack = 0;
            rightFront = Diagonal;
            leftBack = Diagonal;
            leftFront = 0;
        }

        if ( (angle >= ((19 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((21 * Math.PI)/ 16) * (180 / Math.PI)) ) //Diagonal Up Left
        {
            rightFront = 0;
            rightBack = -Diagonal;
            leftFront = -Diagonal;
            leftBack = 0;
        }

        if ( (angle >= ((27 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((29 * Math.PI)/ 16) * (180 / Math.PI)) ) //Diagonal Up Left
        {
            rightBack = 0;
            rightFront = -Diagonal;
            leftBack = -Diagonal;
            leftFront = 0;
        }

        if ( ( (angle >= ((5 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((11 * Math.PI)/ 16) * (180 / Math.PI)) )
            ||  ( (angle >= ((21 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((27 * Math.PI)/ 16) * (180 / Math.PI)) ) )
        {
            rightBack = Vertical;
            rightFront = Vertical;
            leftBack = Vertical;
            leftFront = Vertical;
        }

        if ( ( (angle >= ((13 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((19 * Math.PI)/ 16) * (180 / Math.PI)) )
            || ( (angle >= ((0 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((3 * Math.PI)/ 16) * (180 / Math.PI)) )
                || ( (angle >= ((29 * Math.PI)/ 16) * (180 / Math.PI)) && (angle <= ((32 * Math.PI)/ 16) * (180 / Math.PI)) ) )

        {
            rightBack = Horizontal;
            rightFront = Horizontal;
            leftBack = Horizontal;
            leftFront = Horizontal;
        }

        //assign motors their values
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftBackMotor.setPower(leftBack);
        rightBackMotor.setPower(rightBack);

        //telemetry
        telemetry.addData("Left Front Motor power: ", leftFront);
        telemetry.addData("Right Front Motor power: ", rightFront);
        telemetry.addData("Left Back Motor power: ", leftBack);
        telemetry.addData("Right Back Motor power: ", rightBack);

    }
}