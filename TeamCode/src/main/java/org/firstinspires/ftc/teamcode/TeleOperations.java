package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Robotics on 11/4/2016.
 */
@TeleOp
public class TeleOperations extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor liftMotor;
    DcMotor sweeperMotor;
    DcMotor launchMotor;

    GyroSensor gyro;

    //andymark motor specs
    int ticksPerRevolutionAndy = 1120;
    int maxRPMAndy = 129;
    int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;
    //tertix motor specs
    int ticksPerRevolutionTertix = 1440;
    int maxRPMTetrix = 142;
    int maxTicksPerSecondTetrix = maxRPMTetrix * ticksPerRevolutionTertix;


    public void init(){
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        liftMotor = hardwareMap.dcMotor.get("liftM");
        sweeperMotor = hardwareMap.dcMotor.get("sweepM");

        launchMotor = hardwareMap.dcMotor.get("launchM");

        gyro = hardwareMap.gyroSensor.get("gyro");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sweeperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);
        liftMotor.setMaxSpeed(maxTicksPerSecondTetrix);
        sweeperMotor.setMaxSpeed(maxTicksPerSecondTetrix);
        launchMotor.setMaxSpeed(maxTicksPerSecondAndy);
        // can be brake or float
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void start(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.calibrate();
        while(gyro.isCalibrating()){}

        gyro.resetZAxisIntegrator();
    }

    public void loop(){
        double leftMotorPower = gamepad1.left_stick_y;
        double rightMotorPower = gamepad2.right_stick_y;
        double sweeperPower = gamepad2.left_stick_y;


        if (gamepad2.y)
            liftMotor.setPower(0.5);
        else if (gamepad2.b)
            liftMotor.setPower(-0.5);
        else
            liftMotor.setPower(0);

        if (gamepad2.a) {
            launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int startPos = launchMotor.getCurrentPosition();
            launchMotor.setPower(.8);
            while(launchMotor.getCurrentPosition() < startPos+ticksPerRevolutionAndy){
                launchMotor.setPower(.8);
            }
            launchMotor.setPower(0);
        } else
        launchMotor.setPower(0);


        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);
        sweeperMotor.setPower(sweeperPower);

        telemetry.addData("Gyro", gyro.getHeading());
        telemetry.addData("Left Motor Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position", rightMotor.getCurrentPosition());
        telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
        telemetry.addData("Sweeper Position", sweeperMotor.getCurrentPosition());
        telemetry.addData("Launcher Position", launchMotor.getCurrentPosition());
    }
}
