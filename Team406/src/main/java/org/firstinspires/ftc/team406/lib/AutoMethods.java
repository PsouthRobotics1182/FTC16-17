package org.firstinspires.ftc.team406.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class AutoMethods extends DriveSystem{

    public DcMotor liftMotor;
    DcMotor sweeperMotor;
    DcMotor launchMotor;

    public CRServo button;

    ColorSensor color;

    public void autoSetup(){

        liftMotor = hardwareMap.dcMotor.get("liftM");
        sweeperMotor = hardwareMap.dcMotor.get("sweepM");
        launchMotor = hardwareMap.dcMotor.get("launchM");

        button = hardwareMap.crservo.get("serv");

        color = hardwareMap.colorSensor.get("color");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);



        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //liftMotor.setMaxSpeed(maxTicksPerSecondTetrix);
        sweeperMotor.setMaxSpeed(DriveSystem.maxTicksPerSecondAndy);
        launchMotor.setMaxSpeed(DriveSystem.maxTicksPerSecondAndy60);
        // can be brake or float
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void shoot() throws InterruptedException{

        telemetry.clearAll();
        int startPos = launchMotor.getCurrentPosition();

        while (opModeIsActive() && launchMotor.getCurrentPosition() < startPos + (DriveSystem.ticksPerRevolutionAndy60 * 1.1)) {
            telemetry.addData("Shooting", "In Progress");
            telemetry.update();
            launchMotor.setPower(.4);
            idle();
        }
        launchMotor.setPower(0);
        telemetry.addData("Shooting", "Complete");

    }

    public void pressButton(String target) throws InterruptedException{

        telemetry.clearAll();

        int blue = color.blue();
        int red = color.red();
        telemetry.addData("Color Blue", blue);
        telemetry.addData("Color Red", red);
        telemetry.update();
        if (target.equals("red")) {
            if (red > blue) {
                button.setPower(-1);
                sleep(2000);
                button.setPower(0);
                drive(200, 0.5);
                driveR(150, 0.5);

                blue = color.blue();
                red = color.red();
                telemetry.addData("Color Blue", blue);
                telemetry.addData("Color Red", red);
                telemetry.update();
                if (blue > red)
                    drive(200, 0.5);
            } else if (blue > red) {
                button.setPower(1);
                sleep(2000);
                button.setPower(0);
                drive(200, 0.5);
                driveR(150, 0.5);

                blue = color.blue();
                red = color.red();
                telemetry.addData("Color Blue", blue);
                telemetry.addData("Color Red", red);
                telemetry.update();
                if (blue > red)
                    drive(200, 0.5);
            } else {
                telemetry.clearAll();
                telemetry.addData("Could not decide on color", "");
                telemetry.update();
                pivotLeft(0.2);
                pivotRight(0.2);


                if (red > blue) {
                    button.setPower(-1);
                    sleep(2000);
                    button.setPower(0);
                    drive(200, 0.5);
                    driveR(150, 0.5);

                    blue = color.blue();
                    red = color.red();
                    telemetry.addData("Color Blue", blue);
                    telemetry.addData("Color Red", red);
                    telemetry.update();

                    if (blue > red)
                        drive(200, 0.5);
                } else if (blue > red) {
                    button.setPower(1);
                    sleep(2000);
                    button.setPower(0);
                    drive(200, 0.5);
                    driveR(150, 0.5);

                    blue = color.blue();
                    red = color.red();
                    telemetry.addData("Color Blue", blue);
                    telemetry.addData("Color Red", red);
                    telemetry.update();

                    if (blue > red)
                        drive(200, 0.5);
                } else {
                    telemetry.clearAll();
                    telemetry.addData("Could not decide on color", "");
                    telemetry.update();
                }

            }
        } else if (target.equals("blue")){
            if (blue > red) {
                button.setPower(-1);
                sleep(2000);
                button.setPower(0);
                drive(200, 0.5);
                driveR(150, 0.5);

                blue = color.blue();
                red = color.red();
                telemetry.addData("Color Blue", blue);
                telemetry.addData("Color Red", red);
                telemetry.update();

                if (red > blue)
                    drive(200, 0.5);
            } else if (red > blue) {
                button.setPower(1);
                sleep(2000);
                button.setPower(0);
                drive(200, 0.5);
                driveR(150, 0.5);

                blue = color.blue();
                red = color.red();
                telemetry.addData("Color Blue", blue);
                telemetry.addData("Color Red", red);
                telemetry.update();

                if (red > blue)
                    drive(200, 0.5);
            } else {
                telemetry.clearAll();
                telemetry.addData("Could not decide on color", "");
                telemetry.update();

                pivotLeft(0.2, 0.2);
                pivotRight(0.2, 0.2);


                if (blue > red) {
                    button.setPower(-1);
                    sleep(2000);
                    button.setPower(0);
                    drive(200, 0.5);
                    driveR(150, 0.5);

                    blue = color.blue();
                    red = color.red();
                    telemetry.addData("Color Blue", blue);
                    telemetry.addData("Color Red", red);
                    telemetry.update();

                    if (red > blue)
                        drive(200, 0.5);
                } else if (red > blue) {
                    button.setPower(1);
                    sleep(2000);
                    button.setPower(0);
                    drive(200, 0.5);
                    driveR(150, 0.5);

                    blue = color.blue();
                    red = color.red();
                    telemetry.addData("Color Blue", blue);
                    telemetry.addData("Color Red", red);
                    telemetry.update();

                    if (red > blue)
                        drive(200, 0.5);
                } else {
                    telemetry.clearAll();
                    telemetry.addData("Could not decide on color", "");
                    telemetry.update();
                }

            }
        }
    }

}
