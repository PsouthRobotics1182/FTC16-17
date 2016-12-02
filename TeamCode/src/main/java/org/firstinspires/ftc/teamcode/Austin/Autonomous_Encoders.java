package org.firstinspires.ftc.teamcode.Austin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Robotics on 11/21/2016.
 */
//@Autonomous(name = "Encoder Test", group = "LinearOpMode")
public class Autonomous_Encoders extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor elevator;
    DcMotor popper;


    //andymark motor specs
    int ticksPerRevolutionAndy = 1120;
    int maxRPMAndy = 129;
    int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;
    //tertix motor specs
    int ticksPerRevolutionTertix = 1440;
    int maxRPMTetrix = 142;
    int maxTicksPerSecondTetrix = maxRPMTetrix * ticksPerRevolutionTertix;


    public void runOpMode() throws InterruptedException {

        initializeRobot();
        waitForStart();
        setEncoders();


        while (leftMotor.getCurrentPosition() < MMtoTicks(210))
            drivePower(0.5);

        drivePower(0);

        runElevator();

        while (popper.getCurrentPosition() < ticksPerRevolutionAndy)
            popper.setPower(1.0);

        popper.setPower(0);

        rotate(90);





    }

    //Robot Methods
    public void drivePower(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void runElevator() throws InterruptedException
    {
        elevator.setPower(1.0);
        sleep(1000);
        elevator.setPower(0);
    }

    public void rotate(double degrees)
    {
        double radians = degrees * (180 / Math.PI);
        double ticks = MMtoTicks(115) * radians;

        while (leftMotor.getCurrentPosition() < ticks)
        {
            leftMotor.setPower(1.0);
            rightMotor.setPower(-1.0);
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

        double circ = 3.24459259 * 101.6;
        double mm = revolutions * circ;
        return mm;
    }

    public double MMtoTicks(int mm) {
        double circ = 3.24459259 * 101.6;
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

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

