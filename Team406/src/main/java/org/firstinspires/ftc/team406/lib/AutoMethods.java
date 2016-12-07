package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AutoMethods extends LinearOpMode{

    @Override
    public void runOpMode(){}

    DcMotor liftMotor;
    DcMotor sweeperMotor;
    DcMotor launchMotor;

    CRServo button;

    ColorSensor color;

    public AutoMethods(){

        liftMotor = hardwareMap.dcMotor.get("liftM");
        sweeperMotor = hardwareMap.dcMotor.get("sweepM");
        launchMotor = hardwareMap.dcMotor.get("launchM");

        button = hardwareMap.crservo.get("serv");

        color = hardwareMap.colorSensor.get("color");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //liftMotor.setMaxSpeed(maxTicksPerSecondTetrix);
        sweeperMotor.setMaxSpeed(DriveSystem.maxTicksPerSecondAndy);
        launchMotor.setMaxSpeed(DriveSystem.maxTicksPerSecondAndy);
        // can be brake or float
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void shoot(){
        int startPos = launchMotor.getCurrentPosition();


        while (launchMotor.getCurrentPosition() < startPos + (DriveSystem.ticksPerRevolutionAndy * 1.2)) {
            telemetry.addData("Shooting", "In Progress");
            telemetry.update();
            launchMotor.setPower(.4);
        }
        launchMotor.setPower(0);
        telemetry.addData("Shooting", "Complete");

    }

    public void pressButton(DriveSystem drive, String target) throws InterruptedException{

        button.setPower(-1);
        sleep(1000);
        button.setPower(0);
        int blue = color.blue();
        int red = color.red();
        telemetry.addData("Color Blue", blue);
        telemetry.addData("Color Red", red);
        telemetry.update();
        if (target.equals("red")) {
            if (red - blue > 5) {
                button.setPower(-1);
                sleep(1000);
                button.setPower(0);
                drive.drive(200, 0.5);
                drive.driveR(150, 0.5);

                blue = color.blue();
                red = color.red();
                telemetry.addData("Color Blue", blue);
                telemetry.addData("Color Red", red);
                telemetry.update();

                if (blue - red > 5)
                    drive.drive(200, 0.5);
            } else if (blue - red > 5) {
                button.setPower(1);
                sleep(1000);
                button.setPower(0);
                drive.drive(200, 0.5);
                drive.driveR(150, 0.5);

                blue = color.blue();
                red = color.red();
                telemetry.addData("Color Blue", blue);
                telemetry.addData("Color Red", red);
                telemetry.update();

                if (blue - red > 5)
                    drive.drive(200, 0.5);
            } else {
                telemetry.clearAll();
                telemetry.addData("Could not decide on color", "");
                telemetry.update();

                drive.pivotLeft(0.2);
                drive.pivotRight(0.2);


                if (red - blue > 5) {
                    button.setPower(-1);
                    sleep(1000);
                    button.setPower(0);
                    drive.drive(200, 0.5);
                    drive.driveR(150, 0.5);

                    blue = color.blue();
                    red = color.red();
                    telemetry.addData("Color Blue", blue);
                    telemetry.addData("Color Red", red);
                    telemetry.update();

                    if (blue - red > 5)
                        drive.drive(200, 0.5);
                } else if (blue - red > 5) {
                    button.setPower(1);
                    sleep(1000);
                    button.setPower(0);
                    drive.drive(200, 0.5);
                    drive.driveR(150, 0.5);

                    blue = color.blue();
                    red = color.red();
                    telemetry.addData("Color Blue", blue);
                    telemetry.addData("Color Red", red);
                    telemetry.update();

                    if (blue - red > 5)
                        drive.drive(200, 0.5);
                } else {
                    telemetry.clearAll();
                    telemetry.addData("Could not decide on color", "");
                    telemetry.update();
                }

            }
        } else if (target.equals("blue")){
            if (blue - red > 5) {
                button.setPower(-1);
                sleep(1000);
                button.setPower(0);
                drive.drive(200, 0.5);
                drive.driveR(150, 0.5);

                blue = color.blue();
                red = color.red();
                telemetry.addData("Color Blue", blue);
                telemetry.addData("Color Red", red);
                telemetry.update();

                if (red - blue > 5)
                    drive.drive(200, 0.5);
            } else if (red - blue > 5) {
                button.setPower(1);
                sleep(1000);
                button.setPower(0);
                drive.drive(200, 0.5);
                drive.driveR(150, 0.5);

                blue = color.blue();
                red = color.red();
                telemetry.addData("Color Blue", blue);
                telemetry.addData("Color Red", red);
                telemetry.update();

                if (red - blue > 5)
                    drive.drive(200, 0.5);
            } else {
                telemetry.clearAll();
                telemetry.addData("Could not decide on color", "");
                telemetry.update();

                drive.pivotLeft(0.2, 0.2);
                drive.pivotRight(0.2, 0.2);


                if (blue - red > 5) {
                    button.setPower(-1);
                    sleep(1000);
                    button.setPower(0);
                    drive.drive(200, 0.5);
                    drive.driveR(150, 0.5);

                    blue = color.blue();
                    red = color.red();
                    telemetry.addData("Color Blue", blue);
                    telemetry.addData("Color Red", red);
                    telemetry.update();

                    if (red - blue > 5)
                        drive.drive(200, 0.5);
                } else if (red - blue > 5) {
                    button.setPower(1);
                    sleep(1000);
                    button.setPower(0);
                    drive.drive(200, 0.5);
                    drive.driveR(150, 0.5);

                    blue = color.blue();
                    red = color.red();
                    telemetry.addData("Color Blue", blue);
                    telemetry.addData("Color Red", red);
                    telemetry.update();

                    if (red - blue > 5)
                        drive.drive(200, 0.5);
                } else {
                    telemetry.clearAll();
                    telemetry.addData("Could not decide on color", "");
                    telemetry.update();
                }

            }
        }
    }

}
