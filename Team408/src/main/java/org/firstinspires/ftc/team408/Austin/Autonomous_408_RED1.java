package org.firstinspires.ftc.team408.Austin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Robotics on 11/21/2016.
 */
@Autonomous(name = "408 Auton RED", group = "LinearOpMode")
@Disabled
public class Autonomous_408_RED1 extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor elevator;
    DcMotor popper;

    ColorSensor color;

    Servo buttonPush;


    //andymark motor specs
    int ticksPerRevolutionAndy = 1120;
    int maxRPMAndy = 129;
    int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;



    public void runOpMode() throws InterruptedException {

        initializeRobot();
        waitForStart();
        setEncoders();


        while (leftMotor.getCurrentPosition() < MMtoTicks(210)) {
            drivePower(0.5);
        }

        drivePower(0);
        sleep(1000);



        popper.setPower(-1);
        sleep(1000);

        popper.setPower(0);

        runElevator();

        popper.setPower(-1);
        sleep(1000);

        popper.setPower(0);

        rotate(Math.PI / 4);

        double ticks = leftMotor.getCurrentPosition() + MMtoTicks(700);

        while (leftMotor.getCurrentPosition() < ticks) {
            drivePower(0.5);
        }
        drivePower(0);

        rotate(Math.PI / 2.5);

        ticks = leftMotor.getCurrentPosition() + MMtoTicks(175);

        while (leftMotor.getCurrentPosition() < ticks) {
            drivePower(0.5);
        }
        drivePower(0);

        int c = 0;

        while (opModeIsActive())  {



            telemetry.addData("Red  ", color.red());
            telemetry.addData("Blue ", color.blue());

            if (color.blue() > color.red()) {
                telemetry.addData("Blue", "");
                telemetry.update();
                buttonPush.setPosition(0.0);
                ticks = leftMotor.getCurrentPosition() + MMtoTicks(100);

                drivePower(0.0);
                sleep(1000);

                while (leftMotor.getCurrentPosition() < ticks) {
                    drivePower(0.25);
                }
                drivePower(0);
                break;


            }

            if (color.blue() < color.red()) {
                telemetry.addData("Red", "");
                telemetry.update();
                buttonPush.setPosition(1.0);
                ticks = leftMotor.getCurrentPosition() + MMtoTicks(100);

                drivePower(0.0);
                sleep(1000);

                while (leftMotor.getCurrentPosition() < ticks) {
                    drivePower(0.25);
                }
                drivePower(0);
                break;

            }
            telemetry.update();

            if (c== 2000)
            {
                break;
            }

            c++;

            idle(); //Always end a while (opModeIsActive()) loop with an idle();
        }



        drivePower(-1.0);
        sleep(2000);

        rotate(Math.PI);

        drivePower(0);



    }












    //Robot Methods
    public void drivePower(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(power );
    }

    public void runElevator() throws InterruptedException
    {
        elevator.setPower(1.0);
        sleep(4000);
        elevator.setPower(0);
    }

    public void rotate(double radians)
    {
        double radius =  (16 / 5) * 228.6; //radius 9 inches in MM
        double arc = radians * radius;

        double ticks = leftMotor.getCurrentPosition() + MMtoTicks((int) arc);

        while (leftMotor.getCurrentPosition() < ticks)
        {
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }


    public void rotateRight(double radians)
    {
        double radius =  (16 / 5) * 228.6; //radius 9 inches in MM
        double arc = radians * radius;

        double ticks = rightMotor.getCurrentPosition() + MMtoTicks((int) arc);

        while (rightMotor.getCurrentPosition() < ticks)
        {
            leftMotor.setPower(-1.0);
            rightMotor.setPower(1.0);
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





    //Initialization Methods
    public void initializeRobot() {
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");
        elevator = hardwareMap.dcMotor.get("elevator");
        popper = hardwareMap.dcMotor.get("popper");
        color = hardwareMap.colorSensor.get("color");
        buttonPush = hardwareMap.servo.get("buttonPush");

        color.enableLed(false);

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        popper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);
        popper.setMaxSpeed(maxTicksPerSecondAndy);

        // can be brake or float
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        popper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Ready to Start", "");
        telemetry.update();
    }

    public void setEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        popper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        popper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}

