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

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);
        //liftMotor.setMaxSpeed(maxTicksPerSecondTetrix);
        sweeperMotor.setMaxSpeed(maxTicksPerSecondAndy);
        launchMotor.setMaxSpeed(maxTicksPerSecondAndy);
        // can be brake or float
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void start(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*gyro.calibrate();
        while(gyro.isCalibrating()){}*/

        //gyro.resetZAxisIntegrator();

    }
    double liftPower = 0;
    public void loop() {
        double leftMotorPower = gamepad1.left_stick_y;
        double rightMotorPower = gamepad1.right_stick_y;
        double liftPower = gamepad2.left_stick_y;


        leftMotorPower = scale(leftMotorPower);
        rightMotorPower = scale(rightMotorPower);
        liftPower = scale(liftPower);

        if (gamepad2.a) {
            //launchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int startPos = launchMotor.getCurrentPosition();
            //launchMotor.setTargetPosition((int) (startPos+(ticksPerRevolutionAndy* 0.5)));
            //launchMotor.setPower(.8);


            while (launchMotor.getCurrentPosition() < startPos + (ticksPerRevolutionAndy * 0.5)) {
                telemetry.addData("Cocking", "In Progress");
                telemetry.update();
                launchMotor.setPower(.4);
            }
            launchMotor.setPower(0);
            telemetry.addData("Cocking", "Complete");
        }

        if (gamepad1.right_bumper) {
            sweeperMotor.setPower(-0.5);
        } else if (gamepad1.left_bumper) {
            sweeperMotor.setPower(0.5);
        } else
            sweeperMotor.setPower(0);




        /*liftPower += gamepad2.left_stick_y;

        if (gamepad1.right_trigger > 0.05){
            liftPower += gamepad1.right_trigger;
        //liftMotor.setPower(gamepad1.right_trigger);
        }else if (gamepad1.left_trigger > 0.05) {
            liftPower -= gamepad1.left_trigger;
            //liftMotor.setPower(gamepad1.left_trigger);
        }else
            liftPower += 0;*/

//        sweeperMotor.setPower(sweeperPower);
        if (gamepad1.left_trigger > 0.5){
            leftMotorPower = leftMotorPower/2;
            rightMotorPower = rightMotorPower/2;
        }


        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);
        liftMotor.setPower(liftPower);


        /*telemetry.addData("Gyro", gyro.getHeading());
        telemetry.addData("Left Motor Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position", rightMotor.getCurrentPosition());
        //telemetry.addData("Lift Position", liftMotor.getCurrentPosition());*/
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Lift Power", liftPower + "/" + liftMotor.getPower());
//        telemetry.addData("Sweeper Position", sweeperMotor.getCurrentPosition());
//        telemetry.addData("Sweeper Power", sweeperPower);
//        telemetry.addData("Lift Power", liftPower + "/" + liftMotor.getPower());
//        telemetry.addData("Launcher Position", launchMotor.getCurrentPosition());

        //telemetry.addData("Launcher Power", shooterPower + "/" + gamepad2.right_stick_y + "/" + launchMotor.getPower());
    }

    public double scale(double power){
        if (power < 0)
            power = -(power * power);
        else
            power = power * power;
        return power;
    }
}
