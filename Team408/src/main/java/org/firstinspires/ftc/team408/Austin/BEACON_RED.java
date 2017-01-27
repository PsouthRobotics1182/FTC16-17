/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.team408.Austin;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BEACON RED", group = "LinearOpMode")
public class BEACON_RED extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor popper;

    Servo ball;
    Servo buttonPusher;

    LightSensor lightSensor;  // Hardware Device Object
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor color;

    Servo buttonPush;
    double light;
    double sonarDist;

    final static private double LIGHT_ON_LINE = 1.77;
    final static private double LIGHT_OFF_LINE = 1.3;
    final static private double LIGHT_MARGIN_OF_ERROR = LIGHT_ON_LINE - ((LIGHT_ON_LINE - LIGHT_OFF_LINE) / 2);

    final static private double FULL_POWER = 1;
    final static private double HALF_POWER = 0.5;
    final static private double QUARTER_POWER = 0.25;

    final static private double RIGHT = 0.0;
    final static private double LEFT = 1.0;

    final static private double OUT = 0.6;
    final static private double IN = 0.0;


    //andymark motor specs
    int ticksPerRevolutionAndy = 1120;
    int maxRPMAndy = 129;
    int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;



    public void runOpMode() throws InterruptedException {

        initializeRobot();
        waitForStart();
        setEncoders();


        while (opModeIsActive()) {

            //rotates to the right about 45 degrees
            ball.setPosition(OUT);
            buttonPusher.setPosition(LEFT);

            lineThenButton("RED");
            shootBalls();

            rotateRight(Math.PI / 2, HALF_POWER);
            buttonPusher.setPosition(LEFT);

            lineThenButton("RED");

            break;


        }
    }





    //Robot Methods

    public void hitBothButtons() throws InterruptedException
    {
        lineThenButton("Blue");

        rotateLeft(Math.PI / 2, HALF_POWER);
        buttonPusher.setPosition(LEFT);

        lineThenButton("Blue");

    }

    public void lineThenButton(String color) throws InterruptedException
    {
        driveStraightFor(100, HALF_POWER);
        goUntilLine(HALF_POWER);

        //stops the robot
        drivePower(0);
        sleep(200);

        followLine(QUARTER_POWER);
        hitButton(color);
    }




    public void goUntilLine(double power) throws InterruptedException
    {
        while (opModeIsActive()) {
            light = lightSensor.getRawLightDetected();
            getTelemetry();


            if (light > LIGHT_MARGIN_OF_ERROR) {
                break;
            } else if (light <= LIGHT_MARGIN_OF_ERROR) {
                drivePower(power);
            }
            idle();
        }
    }

    public void followLine(double power) throws InterruptedException
    {
        while (opModeIsActive())
        {
            light = lightSensor.getRawLightDetected();
            sonarDist = rangeSensor.rawUltrasonic();
            getTelemetry();


            if (light > LIGHT_MARGIN_OF_ERROR) {
                drivePower(power);

            } else if (light <= LIGHT_MARGIN_OF_ERROR) {
                turnLeft(power);
            }

            if (sonarDist < 13) {
                drivePower(0);
                break;

            }
            idle();
        }
    }

    public void hitButton(String colorDesired) throws InterruptedException
    {
        if (colorDesired.equals("Red"))
        {
            if (color.blue() > color.red()) {
                hitLeftButton();

            }
            else if (color.blue() < color.red()){
                hitRightButton();
            }

        }
        if (colorDesired.equals("Blue"))
        {
            if (color.blue() > color.red()) {
                hitRightButton();
            }
            else if (color.blue() < color.red()){
                hitLeftButton();
            }
        }
    }

    public void hitLeftButton() throws InterruptedException
    {
        buttonPusher.setPosition(LEFT);
        sleep(1000);
        drivePower(HALF_POWER);
        sleep(500);
        drivePower(0);
        sleep(500);
        driveBackwardsFor(300, 0.5);



    }

    public void hitRightButton() throws InterruptedException
    {
        buttonPusher.setPosition(RIGHT);
        sleep(1000);
        drivePower(HALF_POWER);
        sleep(500);
        drivePower(0);
        sleep(500);
        driveBackwardsFor(300, 0.5);

    }



    public void driveStraightFor(int MM, double power)
    {
        int ticks = (int) MMtoTicks(MM) + rightMotor.getCurrentPosition();

        while (rightMotor.getCurrentPosition() < ticks)
        {
            drivePower(power);
        }
    }

    public void driveBackwardsFor(int MM, double power)
    {
        int ticks =  rightMotor.getCurrentPosition() - (int) MMtoTicks(MM);

        while (rightMotor.getCurrentPosition() > ticks)
        {
            drivePower(-power);
        }
    }

    public void shootBalls() throws InterruptedException
    {
        drivePower(0);
        buttonPusher.setPosition(RIGHT);
        sleep(400);

        popper.setPower(1);
        sleep(1000);

        popper.setPower(0);
        ball.setPosition(IN);
        sleep(1000);
        ball.setPosition(OUT);
        sleep(500);

        popper.setPower(1);
        sleep(1350);

        popper.setPower(0);
        buttonPusher.setPosition(LEFT);
    }





    public void getTelemetry()
    {
        telemetry.addData("Light Sensor", light);
        telemetry.addData("Sonar Distance", rangeSensor.rawUltrasonic());
        telemetry.addData("Left Power", leftMotor.getPower());
        telemetry.addData("Right Power", rightMotor.getPower());
        telemetry.addData("Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());
        telemetry.update();
    }

    public void turnRight(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }
    public void turnLeft(double power)
    {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }
    public void drivePower(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }




    public void rotateLeft(double radians, double power) {
        double radius = (203.6) * (0.18750117); //radius 9 inches in MM
        double arc = radians * radius;

        int ticks =  rightMotor.getCurrentPosition() - (int) MMtoTicks((int) arc);

        while (leftMotor.getCurrentPosition() > ticks) {
            turnLeft(power);
            getTelemetry();
            telemetry.addData("Target Positon:", ticks);
            telemetry.update();

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void rotateRight(double radians, double power)
    {
        double radius =  (203.6); //radius 9 inches in MM
        double arc = radians * radius;

        double ticks = leftMotor.getCurrentPosition() + MMtoTicks((int) arc);

        while (leftMotor.getCurrentPosition() < ticks)
        {
            getTelemetry();
            telemetry.addData("Target Positon:", ticks);
            telemetry.update();
            turnRight(power);
        }

        drivePower(0);
    }




    //Encoder User Interface Methods
    public double scale(double power) {
        if (power < 0)
            power = -(power * power);
        else
            power = power * power;
        return power;
    }

    public double ticksToMM(int ticks) {
        double revolutions = (double) ticks * ticksPerRevolutionAndy;

        double circ = 3.1415926535 * 101.6;
        double mm = revolutions * circ;
        return mm;
    }

    public double MMtoTicks(int mm) {
        double circ = 3.1415926535 * 101.6;
        double revolutions = mm / circ;

        double ticks = revolutions * ticksPerRevolutionAndy;
        return ticks;
    }





    //Initialization Methods
    public void initializeRobot() {
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        buttonPusher = hardwareMap.servo.get("buttonPusher");



        popper = hardwareMap.dcMotor.get("popper");
        color = hardwareMap.colorSensor.get("color");
        buttonPush = hardwareMap.servo.get("buttonPusher");
        ball = hardwareMap.servo.get("ball");

        lightSensor = hardwareMap.lightSensor.get("light sensor");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        color = hardwareMap.colorSensor.get("color");

        lightSensor.enableLed(true);
        color.enableLed(false);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);
        popper.setMaxSpeed(maxTicksPerSecondAndy);

        // can be brake or float
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        popper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Ready to Start", "");
        telemetry.update();
    }

    public void setEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        popper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        popper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void unSetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}

