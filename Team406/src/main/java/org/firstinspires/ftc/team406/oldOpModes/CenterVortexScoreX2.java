/*
package org.firstinspires.ftc.team406.OldOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

*
 * Created by Robotics on 11/4/2016.


//@Autonomous(name = "AutoMouse")
public class CenterVortexScoreX2 extends LinearOpMode {

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


    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        liftMotor = hardwareMap.dcMotor.get("liftM");
        sweeperMotor = hardwareMap.dcMotor.get("sweepM");

        launchMotor = hardwareMap.dcMotor.get("launchM");

        gyro = hardwareMap.gyroSensor.get("gyro");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        telemetry.addData("Ready to Start", "");
        telemetry.update();
        waitForStart();

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
        telemetry.addData("Beginning to Drive", "This is driving");
        telemetry.update();
        sleep(1000);
        while (leftMotor.getCurrentPosition() < MMtoTicks(120)){
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
            telemetry.addData("Position", ticksToMM(leftMotor.getCurrentPosition()));
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        telemetry.addData("Shooting", "Now");
        telemetry.update();
        sleep(1000);

        shoot();

        liftMotor.setPower(-0.3);
        sleep(1000);
        liftMotor.setPower(0);
        shoot();
        shoot();
        while (leftMotor.getCurrentPosition() < MMtoTicks(1400)){
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
        }
while (leftMotor.getCurrentPosition() < 2020){
            leftMotor.setPower(0.5);
            rightMotor.setPower(0);
        }
        while (leftMotor.getCurrentPosition() > 1520){
            leftMotor.setPower(-0.5);
            rightMotor.setPower(0);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    public double scale(double power){
        if (power < 0)
            power = -(power * power);
        else
            power = power * power;
        return power;
    }
    public double ticksToMM(int ticks){
        double revolutions = (double) ticks * ticksPerRevolutionAndy;

        double circ = 3.24459259 * 101.6;
        double mm = revolutions * circ;
        return mm;
    }
    public double MMtoTicks(int mm){
        double circ = 3.24459259 * 101.6;
        double revolutions = mm/circ;

        double ticks = revolutions * ticksPerRevolutionAndy;
        return ticks;
    }

    public void shoot(){

        //launchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int startPos = launchMotor.getCurrentPosition();
        //launchMotor.setTargetPosition((int) (startPos+(ticksPerRevolutionAndy* 0.5)));
        //launchMotor.setPower(.8);


        while (launchMotor.getCurrentPosition() < startPos + (ticksPerRevolutionAndy * 1.1)) {
            telemetry.addData("Shooting", "In Progress");
            telemetry.update();
            launchMotor.setPower(.4);
        }
        launchMotor.setPower(0);
        telemetry.addData("Shooting", "Complete");

    }
}
*/
