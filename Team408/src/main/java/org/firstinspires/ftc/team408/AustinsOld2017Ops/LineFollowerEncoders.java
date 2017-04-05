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

package org.firstinspires.ftc.team408.AustinsOld2017Ops;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Line Follower Encoders", group = "LinearOpMode")
@Disabled
public class LineFollowerEncoders extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor popper;

    Servo ball;
    Servo buttonPusher;

    LightSensor lightSensor, lightSensor2;  // Hardware Device Object
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor color;

    Servo buttonPush;
    double light, light2;
    double sonarDist;

    final static private double LIGHT_ON_LINE = 1.94;
    final static private double LIGHT_OFF_LINE = 1.3;
    final static private double LIGHT_MARGIN_OF_ERROR = LIGHT_ON_LINE - ((LIGHT_ON_LINE - LIGHT_OFF_LINE) / 2);

    final static private double FULL_POWER = 1;
    final static private double HALF_POWER = 0.5;
    final static private double QUARTER_POWER = 0.25;

    final static private double RIGHT = 0.0;
    final static private double LEFT = 1.0;

    final static private double OUT = 0.4;
    final static private double IN = 0.0;


    //andymark motor specs
    int ticksPerRevolutionAndy = 1120;
    int maxRPMAndy = 129;
    int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;


    public void runOpMode() throws InterruptedException {

        //Sets everything up and then waits for the start.
        initializeRobot();
        waitForStart();
        setEncoders();

        //Runs while the phone tells it to
        while (opModeIsActive()) {

            //This is so the ball lift and button pusher are stable or out of the way and in the right place
            ball.setPosition(OUT);
            buttonPusher.setPosition(LEFT);

            //Goes until the line and then hits the blue button
            lineThenButton("Blue");
            driveBackwardsFor(275, 0.5);
            rotateLeft(0.025 * Math.PI / 1.7, HALF_POWER);
            shootBalls();

            //We are having it not go as far out so it can shoot better 2-13-17

            //rotates about 90 degrees but not quite which is why 5 is only close to 4
            rotateRight(1.45 * Math.PI / 1.7, HALF_POWER);//Still turns slightly inconsistent, it may be better to over turn than under turn 2-13-17
            //Makes sure the button pusher is in the right place
            buttonPusher.setPosition(LEFT);

            //Goes until the line and then hits the blue button
            lineThenButton2("Blue");//For some reason it turns too far here and I don't know why 2-14-17
            driveBackwardsFor(300, 0.5);//Separates from wall so it doesn't stay on the button


            //This is the new cap ball part
            rotateRight(1.1 * Math.PI / 1.7, HALF_POWER);
            driveBackwardsFor(1100, FULL_POWER);

            //Stops the program
            break;


        }
    }


    //Robot Methods

    //Hits the button after finding the line
    public void lineThenButton(String color) throws InterruptedException {
        driveStraightFor(100, HALF_POWER);
        //Hits the first line and slows down until it hist the second line and stops
        goUntilLine(HALF_POWER);
        goUntilLine2(0.1);

        //stops the robot
        drivePower(0);
        sleep(200);

        //follows the line and hits the right button
        turnLine(HALF_POWER);//Quarter power so it goes faster than tenth power 2-14-17
        downLine(QUARTER_POWER);
        hitButton(color);
    }

    public void lineThenButton2(String color) throws InterruptedException {
        driveStraightFor(100, HALF_POWER);
        //Hits the first line and slows down until it hist the second line and stops
        goUntilLine(HALF_POWER);
        goUntilLine2(0.1);

        //stops the robot
        drivePower(0);
        sleep(200);

        //follows the line and hits the right button
        turnLine2(HALF_POWER);//Quarter power so it goes faster than tenth power 2-14-17
        downLine(QUARTER_POWER);
        hitButton(color);
    }

    //The robot goes forward until it hits the line
    public void goUntilLine(double power) throws InterruptedException {
        while (opModeIsActive()) {
            light = lightSensor.getRawLightDetected();
            getTelemetry();
            sonarDist = rangeSensor.rawUltrasonic();

            //Detects the yoga ball, and pushes it out of the way 2-21-17
            if (sonarDist <= 15) //This is because at the rolla qualifier the robot would sometimes run into the
            { //Yoga ball here and this would make the entire last 40 points worthless, so this should fix it
                sleep(250);
                drivePower(0);
                sleep(500);
            }
            //When it sees the line it stops, otherwise it drives forward
            if (light > LIGHT_MARGIN_OF_ERROR) {
                drivePower(0);
                sleep(100);
               /* drivePower(-QUARTER_POWER);//The new added weight makes the robot go to far over the line due to momentum so I'm making it back up
                sleep(100);//So that it lines up right
                drivePower(0);
                sleep(100);*/
                break;
            } else if (light <= LIGHT_MARGIN_OF_ERROR) {
                drivePower(power);
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }

    //The robot goes forward until the second light sensor hits the line
    public void goUntilLine2(double power) throws InterruptedException {
        while (opModeIsActive()) {
            light = lightSensor2.getRawLightDetected();
            getTelemetry();

            //When it sees the line it stops, otherwise it drives forward
            if (light > 1.9) {
                break;
            } else if (light <= LIGHT_MARGIN_OF_ERROR) {
                drivePower(power);
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }

    //Rotates the robot until it is square and then follows the line until it is a certain distance away from the wall
    public void turnLine(double power) throws InterruptedException {
        while (opModeIsActive()) {
            light = lightSensor.getRawLightDetected();
            getTelemetry();


            //If the robot sees the line, it stops, otherwise it turns
            if (light > LIGHT_MARGIN_OF_ERROR) {
                rotateLeft(0.6 * Math.PI / 13, power);//This allows the robot to compensate for the sensitivity of the light sensor

                drivePower(0); //This may give the robot time to catch up 2-13-16
                sleep(100);
                break;
            } else if (light <= LIGHT_MARGIN_OF_ERROR) {
                turnLeft(power);
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }

    public void turnLine2(double power) throws InterruptedException {
        while (opModeIsActive()) {
            light = lightSensor.getRawLightDetected();
            getTelemetry();


            //If the robot sees the line, it stops, otherwise it turns
            if (light > LIGHT_MARGIN_OF_ERROR) {
                rotateLeft(0.6 * Math.PI / 8, power);//This allows the robot to compensate for the sensitivity of the light sensor

                drivePower(0); //This may give the robot time to catch up 2-13-16
                sleep(100);
                break;
            } else if (light <= LIGHT_MARGIN_OF_ERROR) {
                turnLeft(power);
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }

    public void downLine(double power) throws InterruptedException {
        while (opModeIsActive()) {
            //Updates the distance from the wall through each iteration
            sonarDist = rangeSensor.rawUltrasonic();
            getTelemetry();
            //If the robot is 13 cm away from the wall it stops, otherwise it drives forward

            //For some reason the robot will keep turning here instead of driving forward 2-10-17
            drivePower(power);
            if (sonarDist < 16) {
                drivePower(0);
                break;
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }

    public void hitButton(String colorDesired) throws InterruptedException {
        if (colorDesired.equals("Red")) {
            if (color.blue() > color.red()) {
                hitLeftButton();

            } else if (color.blue() < color.red()) {
                hitRightButton();
            }

        }
        if (colorDesired.equals("Blue")) {
            if (color.blue() > color.red()) {
                hitLeftButton();
            } else if (color.blue() < color.red()) {
                hitRightButton();
            }
        } else {
            turnRight(QUARTER_POWER);
        }
    }

    public void hitLeftButton() throws InterruptedException {
        buttonPusher.setPosition(LEFT);
        sleep(1000);
        drivePower(HALF_POWER);
        sleep(500);
        drivePower(0);
        sleep(500);


    }

    public void hitRightButton() throws InterruptedException {

        buttonPusher.setPosition(RIGHT);
        sleep(1000);
        drivePower(HALF_POWER);
        sleep(500);
        drivePower(0);
        sleep(500);


    }


    public void driveStraightFor(int MM, double power) {
        int ticks = (int) MMtoTicks(MM) + rightMotor.getCurrentPosition();

        while (rightMotor.getCurrentPosition() < ticks) {
            drivePower(power);
        }
    }

    public void driveBackwardsFor(int MM, double power) {
        int ticks = rightMotor.getCurrentPosition() - (int) MMtoTicks(MM);

        while (rightMotor.getCurrentPosition() > ticks) {
            drivePower(-power);
        }
    }

    public void shootBalls() throws InterruptedException {
        drivePower(0);
        buttonPusher.setPosition(RIGHT);
        sleep(400);

        popper.setPower(1);
        sleep(1000);

        popper.setPower(0);
        ball.setPosition(0.15);
        sleep(1000);
        ball.setPosition(IN);
        sleep(500);
        ball.setPosition(OUT);
        sleep(500);

        popper.setPower(1);
        sleep(1500);

        popper.setPower(0);
        buttonPusher.setPosition(LEFT);
    }


    public void getTelemetry() {
        telemetry.addData("Light Sensor", light);
        telemetry.addData("Sonar Distance", rangeSensor.rawUltrasonic());
        telemetry.addData("Left Power", leftMotor.getPower());
        telemetry.addData("Right Power", rightMotor.getPower());
        telemetry.addData("Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());
        telemetry.update();
    }

    public void turnRight(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public void turnLeft(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    public void drivePower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }


    public void rotateLeft(double radians, double power) {
        double radius = (203.6); //radius 9 inches in MM
        double arc = radians * radius;

        int ticks = rightMotor.getCurrentPosition() + (int) MMtoTicks((int) arc);

        while (rightMotor.getCurrentPosition() < ticks) {
            turnLeft(power);
            getTelemetry();
            telemetry.addData("Target Positon:", ticks);
            telemetry.update();

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void rotateRight(double radians, double power) {
        double radius = (203.6); //radius 9 inches in MM
        double arc = radians * radius;

        double ticks = rightMotor.getCurrentPosition() - MMtoTicks((int) arc);

        while (rightMotor.getCurrentPosition() > ticks) {
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
        lightSensor2 = hardwareMap.lightSensor.get("light sensor2");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        color = hardwareMap.colorSensor.get("color");

        lightSensor.enableLed(true);
        lightSensor2.enableLed(true);
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
}