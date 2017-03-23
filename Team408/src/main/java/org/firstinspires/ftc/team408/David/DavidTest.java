package org.firstinspires.ftc.team408.David;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

/**
 * Created by Robotics on 3/2/2017.
 */

public class DavidTest extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;


    com.qualcomm.robotcore.hardware.LightSensor lightSensor, lightSensor2;  // Hardware Device Object
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor color;


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
            lightSensor.enableLed(true);
            //while(lightSensor.getLightDetected() == (double i = 2 * 1) * 1){

            //}
          //  break;
        }
    }

    public void initializeRobot() {
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        color = hardwareMap.colorSensor.get("color");

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


        // can be brake or float
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Ready to Start", "");
        telemetry.update();
    }

    public void setEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}
