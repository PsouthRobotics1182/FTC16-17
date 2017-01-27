package org.firstinspires.ftc.team406.tele.op;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Robotics on 11/4/2016.
 */
@TeleOp(name="telefone")
public class TeleOperations extends OpMode {

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor liftMotor;
    DcMotor sweeperMotor;
    DcMotor launchMotor;

    CRServo button;

    ColorSensor color;

    ModernRoboticsI2cRangeSensor range;

    LightSensor lineC;
    LightSensor lineF;

    //andymark motor specs
    int ticksPerRevolutionAndy = 1120;
    int maxRPMAndy = 129;
    int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;
    //tertix motor specs
    int ticksPerRevolutionTertix = 1440;
    int maxRPMTetrix = 142;
    int maxTicksPerSecondTetrix = maxRPMTetrix * ticksPerRevolutionTertix;


    public void init(){
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        liftMotor = hardwareMap.dcMotor.get("liftM");
        sweeperMotor = hardwareMap.dcMotor.get("sweepM");

        launchMotor = hardwareMap.dcMotor.get("launchM");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        button = hardwareMap.crservo.get("serv");

        color = hardwareMap.colorSensor.get("color");

        lineC = hardwareMap.lightSensor.get("lineC");
        lineF = hardwareMap.lightSensor.get("lineF");

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sweeperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);


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
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void start(){
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

        lineC.enableLed(true);
        lineF.enableLed(true);

    }
    boolean press;
    double liftPower = 0;
    public void loop() {
        double leftMotorPower = gamepad1.left_stick_y;
        double rightMotorPower = gamepad1.right_stick_y;
        double liftPower = gamepad2.left_stick_y;

        //leftMotorPower = scale(leftMotorPower);
        //rightMotorPower = scale(rightMotorPower);
        liftPower = liftPower/3;

        if (gamepad1.dpad_up)
            liftPower = 0.3;
        else if (gamepad1.dpad_down)
            liftPower = -0.3;
        if(gamepad2.left_bumper)
            button.setPower(-1);
        else if (gamepad2.right_bumper)
            button.setPower(1);
        else
            button.setPower(0);

        if (gamepad2.a || gamepad1.a) {
            int startPos = launchMotor.getCurrentPosition();

            while (launchMotor.getCurrentPosition() < startPos + (ticksPerRevolutionAndy * 1.2)) {
                telemetry.addData("Cocking", "In Progress");
                telemetry.update();
                launchMotor.setPower(.6);
            }
            launchMotor.setPower(0);
            telemetry.addData("Cocking", "Complete");
        }

        if (gamepad1.right_bumper) {
            sweeperMotor.setPower(0.5);
        } else if (gamepad1.left_bumper) {
            sweeperMotor.setPower(-0.5);
        } else
            sweeperMotor.setPower(0);

        if (gamepad1.left_trigger > 0.5){
            leftMotorPower = leftMotorPower/4;
            rightMotorPower = rightMotorPower/4;
            telemetry.clearAll();
            telemetry.addData("Half ", "Power");
        }
        if (gamepad1.right_trigger > 0.5){

            //rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            //leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


            leftMotor = hardwareMap.dcMotor.get("rightM");
            rightMotor = hardwareMap.dcMotor.get("leftM");
            telemetry.addData("Reversed", "");
        } else {

            leftMotor = hardwareMap.dcMotor.get("leftM");
            rightMotor = hardwareMap.dcMotor.get("rightM");
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);
        liftMotor.setPower(liftPower);
        String colors;
        if (color.red() - color.blue() > 2)
            colors = "red";
        else if (color.blue() - color.red() > 2)
            colors = "blue";
        else colors = "unknown";

        //telemetry.addData("Gyro", gyro.getHeading());
        telemetry.addData("Lift Power", liftMotor.getPower());
        telemetry.addData("Right Power", rightMotor.getPower() + "/" + rightMotor.getCurrentPosition());
        telemetry.addData("Left Power", leftMotor.getPower() + "/" + leftMotor.getCurrentPosition());
        telemetry.addData("Color", colors);
        telemetry.addData("Line center", lineC.getLightDetected() + " / "  + lineThereCenter());
        telemetry.addData("Line front", lineF.getLightDetected() + " / " + lineThereFront());
        telemetry.addData("Range", range.cmUltrasonic());
        telemetry.addData("Servo Position", button.getPower());
    }

    public double scale(double power){
        if (power < 0)
            power = -(power * power);
        else
            power = power * power;
        return power;
    }

    //checks for line precense
    public boolean lineThereCenter() {
        telemetry.addData("Line There", lineC.getLightDetected());
        return lineC.getLightDetected() > 0.3;
    }
    public boolean lineThereFront() {
        telemetry.addData("Line There", lineF.getLightDetected());
        return lineF.getLightDetected() > 0.3;
    }
    public boolean lineThere(){
        return lineThereCenter();
    }
}
