package org.firstinspires.ftc.team408.Auto;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 3/21/2017.
 */

public class AutoLib extends LinearOpMode
{
    public void runOpMode() throws InterruptedException {}
    //drive motors
    DcMotor lift, lift2, leftMotor, rightMotor, popper, sweeperMotor, liftMotor;

    CRServo buttonPusher, buttonPusherLeft;
    Servo capHand1, capHand2;

    LightSensor lightSensor, lightSensor2;  // Hardware Device Object
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor color;

    //drive values
    double left, right, liftPower;

    int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution;

    final static private double LIGHT_ON_LINE = 2.24; //Initializes my constants
    final static private double LIGHT_OFF_LINE = 1.7;
    final static private double LIGHT_MARGIN_OF_ERROR = LIGHT_ON_LINE - ((LIGHT_ON_LINE - LIGHT_OFF_LINE) / 12);
    //I use the light margin of error double as a basis because it can identify a greater range than simply the
    //typical light value when on the line.
    //We can use light sensors to detect the line becaue white lines reflect more light than a gray field

    final static double FULL_POWER = 1; //Drive speeds
    final static double HALF_POWER = 0.5;
    final static double QUARTER_POWER = 0.25;





    public void config()
    {
        //andymark motor specs
        int ticksPerRevolutionAndy = 1120;
        int maxRPMAndy = 129;
        int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;

        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        buttonPusher = hardwareMap.crservo.get("serv");
        buttonPusherLeft = hardwareMap.crservo.get("serv2");
        popper = hardwareMap.dcMotor.get("launchM");
        lightSensor = hardwareMap.lightSensor.get("lineF");
        lightSensor2 = hardwareMap.lightSensor.get("lineC");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        color = hardwareMap.colorSensor.get("color");
        lift = hardwareMap.dcMotor.get("capBot");
        lift2 = hardwareMap.dcMotor.get("capTop");
        sweeperMotor = hardwareMap.dcMotor.get("sweepM");
        liftMotor = hardwareMap.dcMotor.get("liftM");
        capHand1 = hardwareMap.servo.get("caphand1");
        capHand2 = hardwareMap.servo.get("caphand2");


        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);
        popper.setMaxSpeed(maxTicksPerSecondAndy);
        lift.setMaxSpeed((int) (maxTicksPerSecondAndy * 0.9));
        lift2.setMaxSpeed((int) (maxTicksPerSecondAndy * 0.9));

        setEncoders();

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        popper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        popper.setDirection(DcMotorSimple.Direction.FORWARD);


        lightSensor.enableLed(true);
        lightSensor2.enableLed(true);
        color.enableLed(false);

        capHand1.setPosition(1);
        capHand2.setPosition(0);
    }

    //Robot Methods
    public void setEncoders()
    {
        popper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        popper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Hits the button after finding the line
    public void lineThenButton(String colorName) throws InterruptedException {
        //Hits the first line and slows down until it hist the second line and stops
        goUntilLine(-1.0);
        goUntilLine2(-0.1);
        drivePower(0);
        //follows the line and hits the right button

        if (colorName.equals("Red"))
            turnLineRed(0.1);//Quarter power so it goes faster than tenth power 2-14-17
        else
            turnLineBlue(0.1);


        downLine(-QUARTER_POWER);
        //rotateRight(Math.PI / 12, HALF_POWER);
        hitButton(colorName);
    }

    //The robot goes forward until it hits the line
    public void goUntilLine(double power) throws InterruptedException {
        while (opModeIsActive() && lightSensor.getRawLightDetected() < LIGHT_MARGIN_OF_ERROR) {

            getTelemetry();
            drivePower(power); //Drives straight if it doesn't see the line
            idle(); //Idles give the robot time to catch up with the loop
        }
        drivePower(0);
    }

    //The robot goes forward until the second light sensor hits the line
    public void goUntilLine2(double power) throws InterruptedException {
        while (opModeIsActive() && lightSensor2.getRawLightDetected() < LIGHT_MARGIN_OF_ERROR) {
            getTelemetry();
            drivePower(power);
            idle(); //Idles give the robot time to catch up with the loop
        }
        drivePower(0);
    }

    //Rotates the robot until it is square and then follows the line until it is a certain distance away from the wall
    public void turnLineRed(double power) throws InterruptedException {
        while (opModeIsActive() && lightSensor.getRawLightDetected() < LIGHT_MARGIN_OF_ERROR) {
            getTelemetry();
            turnLeft(power);
            idle(); //Idles give the robot time to catch up with the loop
        }
        /*while (opModeIsActive() && lightSensor.getRawLightDetected() > LIGHT_MARGIN_OF_ERROR) {
            getTelemetry();
            turnLeft(power);
            idle(); //Idles give the robot time to catch up with the loop
        }*/

        //drivePower(0);

        //Turn back to adjust I think
        /*while (opModeIsActive() && lightSensor.getRawLightDetected() < LIGHT_MARGIN_OF_ERROR) {
            getTelemetry();
            turnRight(power);
            idle();
        }*/



        drivePower(0);
    }
    //Rotates the robot until it is square and then follows the line until it is a certain distance away from the wall
    public void turnLineBlue(double power) throws InterruptedException {
        while (opModeIsActive() && lightSensor.getRawLightDetected() < LIGHT_MARGIN_OF_ERROR) {
            getTelemetry();
            turnRight(power);
            idle(); //Idles give the robot time to catch up with the loop
        }
        /*while (opModeIsActive() && lightSensor.getRawLightDetected() > LIGHT_MARGIN_OF_ERROR) {
            getTelemetry();
            turnRight(power);
            idle(); //Idles give the robot time to catch up with the loop
        }*/
        drivePower(0);
    }

    public void downLine(double power) throws InterruptedException {
        //If the robot is 18 cm away from the wall it stops, otherwise it drives forward
        while (opModeIsActive() && rangeSensor.cmUltrasonic() > 20) {
            getTelemetry();
            //For some reason the robot will keep turning here instead of driving forward 2-10-17
            drivePower(power);
            idle(); //Idles give the robot time to catch up with the loop
        }
        drivePower(0);
        sleep(200);
    }

    //Hits the right button
    public void hitButton(String colorDesired) throws InterruptedException {
        if (colorDesired.equals("Red")) {
            if (color.blue() > color.red())
                hitRightButtonRed();
            else
                hitLeftButtonRed();
        }
        if (colorDesired.equals("Blue")) {
            if (color.blue() > color.red())
                hitLeftButtonBlue();
            else
                hitRightButtonBlue();
        }
    }

    //Hits the left button
    public void hitLeftButtonBlue() throws InterruptedException {
        telemetry.addData("HIT LEFT", "");
        telemetry.update();
        rotateRight(Math.PI / 40, HALF_POWER);
        buttonPusher.setPower(1.0);
        sleep(2000);
        drivePower(-QUARTER_POWER);
        sleep(600);
        drivePower(0);
        sleep(100);
        drivePower(QUARTER_POWER);
        sleep(200);
        drivePower(0);
        sleep(100);
        rotateLeft(Math.PI / 40, HALF_POWER);
    }

    //Hits the right button
    public void hitRightButtonBlue() throws InterruptedException {
        telemetry.addData("HIT RIGHT", "");
        telemetry.update();
        rotateLeft(Math.PI / 60, HALF_POWER);
        buttonPusherLeft.setPower(-1.0);
        sleep(2000);
        drivePower(-QUARTER_POWER);
        sleep(600);
        drivePower(0);
        sleep(100);
        drivePower(QUARTER_POWER);
        sleep(200);
        drivePower(0);
        sleep(100);

    }

    //Hits the left button
    public void hitLeftButtonRed() throws InterruptedException {
        telemetry.addData("HIT LEFT", "");
        telemetry.update();
        rotateRight(Math.PI / 25, HALF_POWER);
        buttonPusher.setPower(1.0);
        sleep(2000);
        drivePower(-QUARTER_POWER);
        sleep(300);
        drivePower(0);
        sleep(100);
        drivePower(QUARTER_POWER);
        sleep(200);
        drivePower(0);
        sleep(100);
        rotateLeft(Math.PI / 35, HALF_POWER);
    }

    //Hits the right button
    public void hitRightButtonRed() throws InterruptedException {
        telemetry.addData("HIT RIGHT", "");
        telemetry.update();
        rotateLeft(Math.PI / 25, HALF_POWER);
        buttonPusherLeft.setPower(-1.0);
        sleep(2000);
        drivePower(-QUARTER_POWER);
        sleep(300);
        drivePower(0);
        sleep(100);
        drivePower(QUARTER_POWER);
        sleep(200);
        drivePower(0);
        sleep(100);
        rotateRight(Math.PI / 25, HALF_POWER);


    }
    //Drives forward for a mm distance at a power speed
    public void driveStraightFor(int MM, double power) {
        int ticks = (int) MMtoTicks(MM) + rightMotor.getCurrentPosition();

        while (rightMotor.getCurrentPosition() < ticks) {
            drivePower(power);
        }
    }

    //Drives backwards for a mm distance at a power speed
    public void driveBackwardsFor(int MM, double power) {
        int ticks = rightMotor.getCurrentPosition() - (int) MMtoTicks(MM);

        while (rightMotor.getCurrentPosition() > ticks) {
            drivePower(-power);
        }
    }

    //Shoots the balls
    public void shoot() throws InterruptedException
    {
        int startPos = popper.getCurrentPosition();

       while (opModeIsActive() && popper.getCurrentPosition() < startPos + (1680 * 1.1))
       {
           popper.setPower(1);
       }
        popper.setPower(0);


    }

    public void shootBalls() throws InterruptedException
    {
        shoot();
        liftMotor.setPower(-1);
        sleep(1500);
        liftMotor.setPower(0);
        shoot();
    }


    //Returns telemetry data to the driver station
    public void getTelemetry() {
        telemetry.addData("Sonar Distance", rangeSensor.rawUltrasonic());
        telemetry.addData("Left Power", leftMotor.getPower());
        telemetry.addData("Right Power", rightMotor.getPower());
        telemetry.addData("Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());
        telemetry.update();
    }

    //Turns the robot right at a speed
    public void turnRight(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    //Turns the robot left at a speed
    public void turnLeft(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    //The robot drives straight at a speed power, forward or backwards
    public void drivePower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }


    //Rotates the robot left for an angle at a power
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

    //Rotates the robot right for an angle at a power
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

    //Converts the encoder ticks to mm
    public double ticksToMM(int ticks) {
        double revolutions = (double) ticks * ticksPerRevolution;

        double circ = 3.1415926535 * 101.6;
        double mm = revolutions * circ;
        return mm;
    }

    //Converts the mm to encoder ticks
    public double MMtoTicks(int mm) {
        double circ = 3.1415926535 * 101.6;
        double revolutions = mm / circ;

        double ticks = revolutions * ticksPerRevolution;
        return ticks;
    }

    public void spin()
    {
        while (opModeIsActive()) {
            turnRight(FULL_POWER);
        }
    }
}
