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

package org.firstinspires.ftc.team408.AustinsOld2017Ops.Vasu;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

@Autonomous(name = "Vasu's Test", group = "LinearOpMode")
@Disabled
public class vasu extends LinearOpMode {
    LightSensor lineFollower;  // Hardware Device Object
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor color;

    DcMotor leftMotor;
    DcMotor rightMotor;

    //andymark motor specs
    int ticksPerRevolutionAndy = 1120;
    int maxRPMAndy = 129;
    int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;

    double light;

    @Override
    public void runOpMode() throws InterruptedException {


        lineFollower = hardwareMap.lightSensor.get("light sensor");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        color = hardwareMap.colorSensor.get("color");

        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        lineFollower.enableLed(true);


        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMaxSpeed(1120*129-100);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMaxSpeed(1120*129-100);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //drive forward until the robot sees the line

        rightMotor.setPower(0.5);
        leftMotor.setPower(0.5);
        while(opModeIsActive()&&lineFollower.getRawLightDetected()>=1.45) {} //wait until the robot sees the line
        rightMotor.setPower(0);
        leftMotor.setPower(0);      //turn off the motors


    }


    public void rotate(double radians) {
        double radius = (16 / 5) * 228.6; //radius 9 inches in MM
        double arc = radians * radius;

        double ticks = leftMotor.getCurrentPosition() - MMtoTicks((int) arc);

        while (leftMotor.getCurrentPosition() > ticks) {
            leftMotor.setPower(-1.0);
            rightMotor.setPower(1.0);
            telemetry.addData("Right Motor position: ", leftMotor.getCurrentPosition());
            telemetry.addData("Target Position", ticks);
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void rotateLeft(double radians) {
        double radius = (16 / 5) * 228.6; //radius 9 inches in MM
        double arc = radians * radius;

        double ticks = leftMotor.getCurrentPosition() + MMtoTicks((int) arc);

        while (leftMotor.getCurrentPosition() < ticks) {
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
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

    public void setEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}


