package org.firstinspires.ftc.team408.Austin;

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

    LightSensor lightSensor, lightSensor2;  // Hardware Device Object
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor color;

    //drive values
    double left, right, liftPower;
    double light, light2; //These are holder values
    double sonarDist;

    Boolean speed = true, ballCatch = true, breaks = false;

    int ticksPerRevolution = 1440;
    int maxRPM = 152;
    int maxTicksPerSecond = maxRPM * ticksPerRevolution;

    final static private double LIGHT_ON_LINE = 2; //Initializes my constants
    final static private double LIGHT_OFF_LINE = 1.6;
    final static private double LIGHT_MARGIN_OF_ERROR = LIGHT_ON_LINE - ((LIGHT_ON_LINE - LIGHT_OFF_LINE) / 2);
    //We can use light sensors to detect the line becaue white lines reflect more light than a gray field

    final static private double FULL_POWER = 1; //Drive speeds
    final static private double HALF_POWER = 0.5;
    final static private double QUARTER_POWER = 0.25;

    final static private double RIGHT = 0.0; //Button push positions
    final static private double LEFT = 1.0;

    final static private double OUT = 0.4; //Ball dump positions
    final static private double IN = 0.0;


    public void runRedAuto() throws InterruptedException
    {
        //Runs while the phone tells it to
        while (opModeIsActive()) {

            //This is so the ball lift and button pusher are stable or out of the way and in the right place


            //Goes until the line and then hits the blue button
            lineThenButton("Red");
            buttonPusher.setPower(-1.0);
            driveStraightFor(300, 0.5); //Backs up to get in the right place for shooting
            drivePower(0);
            sleep(1000);
            buttonPusher.setPower(0);
            shootBalls();

            //We are having it not go as far out so it can shoot better 2-13-17

            //rotates about 90 degrees but not quite which is why 5 is only close to 4
            rotateRight(0.67 * Math.PI / 2, HALF_POWER);//Still turns slightly inconsistent, it may be better to over turn than under turn 2-13-17
            //Makes sure the button pusher is in the right place


            //Goes until the line and then hits the blue button
            lineThenButton("Red");//For some reason it turns too far here and I don't know why 2-14-17
            driveStraightFor(300, 0.5);//Separates from wall so it doesn't stay on the button



            //This is the new cap ball part
            rotateRight(Math.PI / 12, HALF_POWER);
            buttonPusher.setPower(-1.0);
            //driveStraightFor(200, FULL_POWER);
            sleep(1000);
            buttonPusher.setPower(0);

            //Stops the program
            break;


        }
    }

    public void runBlueAuto() throws InterruptedException
    {
        //Runs while the phone tells it to
        while (opModeIsActive()) {

            //This is so the ball lift and button pusher are stable or out of the way and in the right place


            //Goes until the line and then hits the blue button
            lineThenButton("Blue");
            buttonPusher.setPower(-1.0);
            driveStraightFor(300, 0.5); //Backs up to get in the right place for shooting
            drivePower(0);
            sleep(1000);
            buttonPusher.setPower(0);
            shootBalls();

            //We are having it not go as far out so it can shoot better 2-13-17
            driveBackwardsFor(50, 0.5);
            //rotates about 90 degrees but not quite which is why 5 is only close to 4
            rotateLeft(0.65 * Math.PI / 2, HALF_POWER);//Still turns slightly inconsistent, it may be better to over turn than under turn 2-13-17
            //Makes sure the button pusher is in the right place


            //Goes until the line and then hits the blue button
            lineThenButton("Blue");//For some reason it turns too far here and I don't know why 2-14-17
            driveStraightFor(300, 0.5);//Separates from wall so it doesn't stay on the button



            //This is the new cap ball part
            rotateLeft(Math.PI / 12, HALF_POWER);
            buttonPusher.setPower(-1.0);
            //driveStraightFor(200, FULL_POWER);
            sleep(1000);
            buttonPusher.setPower(0);

            //Stops the program
            break;


        }
    }



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
        color = hardwareMap.colorSensor.get("color");
        //ball = hardwareMap.servo.get("ball");
        lightSensor = hardwareMap.lightSensor.get("lineF");
        lightSensor2 = hardwareMap.lightSensor.get("lineC");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        color = hardwareMap.colorSensor.get("color");
        lift = hardwareMap.dcMotor.get("capBot");
        lift2 = hardwareMap.dcMotor.get("capTop");
        sweeperMotor = hardwareMap.dcMotor.get("sweepM");
        liftMotor = hardwareMap.dcMotor.get("liftM");


        //Mapping physical motors to variable
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        popper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        popper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        popper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        lightSensor.enableLed(true);
        lightSensor2.enableLed(true);
        color.enableLed(false);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        popper.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);
        popper.setMaxSpeed(maxTicksPerSecondAndy);


        // can be brake or float

        telemetry.addData("Ready to Start", "");

    }

    public void setEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        popper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        popper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Robot Methods

    //Hits the button after finding the line
    public void lineThenButton(String color) throws InterruptedException {
        driveBackwardsFor(100, HALF_POWER);
        //Hits the first line and slows down until it hist the second line and stops
        goUntilLine(-QUARTER_POWER);
        goUntilLine2(-0.1);

        //follows the line and hits the right button

        if (color.equals("Red"))
            turnLineRed(0.1);//Quarter power so it goes faster than tenth power 2-14-17
        else
            turnLineBlue(0.1);


        downLine(-0.2);
        //rotateRight(Math.PI / 12, HALF_POWER);
        hitButton(color);
    }

    //The robot goes forward until it hits the line
    public void goUntilLine(double power) throws InterruptedException {
        while (opModeIsActive()) {
            light = lightSensor.getRawLightDetected(); //Detects the light
            getTelemetry();

            //When it sees the line it stops, otherwise it drives forward
            if (light > LIGHT_MARGIN_OF_ERROR) {
                break;
            } else {
                drivePower(power); //Drives straight if it doesn't see the line
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }

    //The robot goes forward until the second light sensor hits the line
    public void goUntilLine2(double power) throws InterruptedException {
        while (opModeIsActive()) {
            light = lightSensor2.getRawLightDetected(); //Detects the light of the line
            getTelemetry();

            //When it sees the line it stops, otherwise it drives forward
            if (light > LIGHT_MARGIN_OF_ERROR) {
                drivePower(0);
                sleep(100);
                break;
            } else {
                drivePower(power);
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }

    //Rotates the robot until it is square and then follows the line until it is a certain distance away from the wall
    public void turnLineRed(double power) throws InterruptedException {
        while (opModeIsActive()) {
            light = lightSensor.getRawLightDetected();
            getTelemetry();


            //If the robot sees the line, it stops, otherwise it turns
            if (light > LIGHT_MARGIN_OF_ERROR) {

                drivePower(0); //This may give the robot time to catch up 2-13-16
                sleep(100);
                rotateRight(Math.PI / 35, QUARTER_POWER);//Potentially corrects for some error in the line following

                break;
            } else if (light <= LIGHT_MARGIN_OF_ERROR) {
                turnLeft(power);
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }
    //Rotates the robot until it is square and then follows the line until it is a certain distance away from the wall
    public void turnLineBlue(double power) throws InterruptedException {
        while (opModeIsActive()) {
            light = lightSensor.getRawLightDetected();
            getTelemetry();


            //If the robot sees the line, it stops, otherwise it turns
            if (light > LIGHT_MARGIN_OF_ERROR) {

                drivePower(0); //This may give the robot time to catch up 2-13-16
                sleep(100);
                break;
            } else if (light <= LIGHT_MARGIN_OF_ERROR) {
                turnRight(power);
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
            if (sonarDist < 18) {
                drivePower(0);
                break;
            }
            idle(); //Idles give the robot time to catch up with the loop
        }
    }

    //Hits the right button
    public void hitButton(String colorDesired) throws InterruptedException {
        if (colorDesired.equals("Red")) {
            if (color.blue() > color.red())
                hitRightButton();

             else if (color.blue() < color.red())
                hitLeftButton();
        }
        if (colorDesired.equals("Blue")) {
            if (color.blue() > color.red())
                hitLeftButton();
             else if (color.blue() < color.red())
                hitRightButton();
        }
    }

    //Hits the left button
    public void hitLeftButton() throws InterruptedException {
        telemetry.addData("HIT LEFT", "");
        telemetry.update();
        drivePower(-HALF_POWER);
        buttonPusher.setPower(1.0);
        sleep(500);
        drivePower(0);
        sleep(750);
    }

    //Hits the right button
    public void hitRightButton() throws InterruptedException {
        telemetry.addData("HIT RIGHT", "");
        telemetry.update();
        /*buttonPusherLeft.setPower(1.0);
        sleep(1000);
        drivePower(HALF_POWER);
        sleep(1200);
        drivePower(0);
        sleep(500);*/
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
        sleep(1000);
        liftMotor.setPower(0);
        shoot();
    }


    //Returns telemetry data to the driver station
    public void getTelemetry() {
        telemetry.addData("Light Sensor", light);
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
