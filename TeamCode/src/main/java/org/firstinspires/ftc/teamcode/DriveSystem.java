package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/*
 * Created by dzogh_000 on 9/17/2016.
 */
public class DriveSystem extends LinearOpMode {
    //Implementing nessasary methods
    @Override
    public void runOpMode(){}

    DcMotor leftMotor;
    DcMotor rightMotor;

    GyroSensor gyro;

    //variables to run motors using encoders
    int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution;

    DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
    DcMotorSimple.Direction rightDirection, leftDirection;
    public DriveSystem(DcMotorSimple.Direction leftDirection, DcMotorSimple.Direction rightDirection){

        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        gyro = hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();
        while (gyro.isCalibrating()){
            telemetry.addData("Gyro", "Calibrating");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Calibrated");
        telemetry.update();

        this.leftDirection = leftDirection;
        this.rightDirection = rightDirection;

        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);

        leftMotor.setMaxSpeed(maxTicksPerSecond);
        rightMotor.setMaxSpeed(maxTicksPerSecond);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setDirection("FORWARD");
        telemetry.addData("Drive System", "initialized");
        telemetry.update();
    }

    public void pivot(int angle) throws InterruptedException {

        telemetry.addData("Pivot", "starting");
        telemetry.update();

        gyro.resetZAxisIntegrator();

        resetEncoders();

        double integral =  0;
        double prevError = 0;
        double dt = 20;

        double P = 0.002;
        double I = 0.002;
        double D = 0.002;

        if (angle < 0){
            leftMotor.setDirection(rightDirection);
            rightMotor.setDirection(rightDirection);

            angle = 360 + angle;
            //turns left
            //todo fix PID logic for left pivot
            while (gyro.getHeading() > angle || gyro.getHeading() == 0){
                telemetry.addData("Pivot", "in progress");
                telemetry.update();
                double error = gyro.getHeading() - angle;
                //get heading return a value from 0-360butif the but if the value is 340 the should be -20
                //this if fixes that,

                double pro = error * P;
                integral = I * (integral + error * dt);
                double der = D * ((error - prevError)/dt);
                //correction is negitive if vering to left
                double PIDcorrect = pro + integral * der;

                leftMotor.setPower(0.5 - PIDcorrect);
                rightMotor.setPower(0.5 + PIDcorrect);

                prevError = error;

                sleep((long)dt);

            }
            telemetry.addData("Pivot", "complete");
            telemetry.update();
        }
        //pivots right
        if (angle > 0){
            leftMotor.setDirection(leftDirection);
            rightMotor.setDirection(leftDirection);

            while (gyro.getHeading() < angle){
                telemetry.addData("Pivot", "in progress");
                telemetry.update();
                double error = angle - gyro.getHeading();

                double pro = error * P;
                integral = I * (integral + error * dt);
                double der = D * ((error - prevError)/dt);
                //correction is negitive if vering to left
                double PIDcorrect = pro + integral * der;

                leftMotor.setPower(0.5 + PIDcorrect);
                rightMotor.setPower(0.5 - PIDcorrect);

                prevError = error;

                sleep((long)dt);

            }
            telemetry.addData("Pivot", "complete");
            telemetry.update();

        }
        stopMotors();
    }

    public void drive(double power, double distance) throws InterruptedException {

        telemetry.addData("Drive", "starting");
        telemetry.update();
        //reset the gyro so the set point is zero and we dont have to check what the robot angle is
        gyro.resetZAxisIntegrator();

        //variables for PID loopto keep robot driving straight
        double integral =  0;
        double prevError = 0;
        double dt = 20;
        //coeffiecients to tune loop
        double P = 0.002;
        double I = 0.002;
        double D = 0.002;

        //reset encoders
        resetEncoders();
        //parameter to tune how the robot slows as it approaches target distance;
        double pD = 0.00001;
        while (ticksToMM(leftMotor.getCurrentPosition()) < distance){
            telemetry.addData("Drive", "in progress");
            telemetry.update();
            //calculates PID correction
            double error = gyro.getHeading();
            //get heading return a value from 0-360butif the but if the value is 340 the should be -20
            //this if fixes that,
            if (error > 180)
                error = 0 - (360 - error);

            double pro = error * P;
            integral = I * (integral + error * dt);
            double der = D * ((error - prevError)/dt);
            //correction is negitive if vering to left
            double PIDcorrect = pro + integral * der;
            double leftPower = power - PIDcorrect;
            double rightPower = power + PIDcorrect;

            //calculates how the robot willl slow as it approaches destination
            double errorD = distance - ticksToMM(leftMotor.getCurrentPosition());
            double proD = errorD * pD;

            leftPower = leftPower + proD;
            rightPower = rightPower + proD;

            leftPower = Range.clip(leftPower, 0, 1);
            rightPower = Range.clip(rightPower, 0, 1);

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            prevError = error;

            sleep((long)dt);


        }
        stopMotors();
        telemetry.addData("Drive", "complete");
        telemetry.update();
    }
    public double ticksToMM(int ticks){
        double revolutions = (double) ticks * ticksPerRevolution;

        double circ = 3.24459259 * 101.6;
        double mm = revolutions * circ;
        return mm;
    }


    public void setDirection(String direction){
        // // TODO: 9/17/2016 find proper motor directions
        if (direction.equals("FORWARD")){
            leftMotor.setDirection(leftDirection);
            rightMotor.setDirection(rightDirection);
        }
        if (direction.equals("REVERSE")){
            leftMotor.setDirection(rightDirection);
            rightMotor.setDirection(leftDirection);
        }
        if (direction.equals("PIVOT_RIGHT")){
            leftMotor.setDirection(leftDirection);
            rightMotor.setDirection(leftDirection);
        }
        if (direction.equals("PIVOT_LEFT")){
            leftMotor.setDirection(rightDirection);
            rightMotor.setDirection(rightDirection);
        }
    }
    public void stopMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void resetEncoders(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
