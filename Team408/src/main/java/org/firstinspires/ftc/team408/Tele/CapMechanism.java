package org.firstinspires.ftc.team408.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by Robotics on 3/16/2017.
 */

public class CapMechanism {
    public DcMotor capMotorTop;//the upper cap ball motor, 40, encoder
    public DcMotor capMotorBottom;//lower cap ball motor, 40, encoder
    public Servo capHand;
    public Servo capHand2;

    public LinearOpMode opMode;


    public CapMechanism (LinearOpMode opmode, HardwareMap hwMap){
        this.opMode = opmode;
        capMotorTop = hwMap.dcMotor.get("capTop");
        capMotorBottom = hwMap.dcMotor.get("capBot");

        capHand = hwMap.servo.get("caphand1");
        capHand2 = hwMap.servo.get("caphand2");

        capMotorTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capMotorBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        capMotorTop.setDirection(DcMotorSimple.Direction.REVERSE);
        capMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        capMotorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        capMotorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        capMotorTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capMotorBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        capMotorTop.setMaxSpeed(AutoLibV2.maxTicksPerSecondAndy);
        capMotorBottom.setMaxSpeed(AutoLibV2.maxTicksPerSecondAndy);
    }

    public void setPower(double power){
        capMotorTop.setPower(power);
        capMotorBottom.setPower(power);
    }

    public void grab(boolean bool){
        if (bool){
            capHand.setPosition(1);
            capHand2.setPosition(0);
        }else{
            capHand.setPosition(0);
            capHand2.setPosition(1);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        capMotorTop.setZeroPowerBehavior(behavior);
        capMotorBottom.setZeroPowerBehavior(behavior);
    }

    public double getPower(){
        return (capMotorTop.getPower() + capMotorBottom.getPower())/2;
    }
}
