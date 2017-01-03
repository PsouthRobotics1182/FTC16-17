package org.firstinspires.ftc.team406.lib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;

//TODO add opModeIsActive to all loops
public abstract class DriveSystem extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    }

    DcMotor leftMotor;
    DcMotor rightMotor;

    ModernRoboticsI2cRangeSensor range;

    LightSensor line;

    //andymark motor specs
    static int ticksPerRevolutionAndy = 1120;
    static int maxRPMAndy = 129;
    static int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;
    //tertix motor specs
    static int ticksPerRevolutionTertix = 1440;
    static int maxRPMTetrix = 142;
    static int maxTicksPerSecondTetrix = maxRPMTetrix * ticksPerRevolutionTertix;

    public void configure() {
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        range = null;//range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        line = hardwareMap.lightSensor.get("line");

        setDirection("FORWARD");

        //set run mode to use encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);
        //makes it so the motor cant be backdriven easily
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        line.enableLed(true);
    }

    //drives forward a certain distance
    public void drive(int mm, double power) {
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        //runs while encoders give a position less than the intended distance
        while (opModeIsActive() && (leftMotor.getCurrentPosition() < MMtoTicks(mm) && rightMotor.getCurrentPosition() < MMtoTicks(mm))) {
            drive(power);
            telemetry.clearAll();
            telemetry.addData("Driving", "now");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Drive", "Complete");
        telemetry.update();
        stopMotors();
    }

    //drives until sonar reads a certain distance
    public void driveD(int mm, double power) {  //uses mm so it is consistnent with the rest of the methods
        //converts mm to cm
        double cm = ((double) mm) / 100;
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        //runs while the ultasonic sensor reads higher than the desired cm
        while (opModeIsActive() && (range.cmUltrasonic() > cm)) {
            drive(power);
            telemetry.clearAll();
            telemetry.addData("Driving", "now");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Drive", "Complete");
        telemetry.update();
        stopMotors();
    }

    //drives in reverse a certain distance
    public void driveR(int mm, double power) {
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        setDirection("REVERSE");
        //runs until desired distance is reacehed
        while (opModeIsActive() && (leftMotor.getCurrentPosition() < MMtoTicks(mm) && rightMotor.getCurrentPosition() < MMtoTicks(mm))) {
            drive(power);
            telemetry.clearAll();
            telemetry.addData("Driving", "now");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        setDirection("FORWARD");
        telemetry.addData("Drive", "Complete");
        telemetry.update();
        stopMotors();
    }

    //drives until it sees a line
    public void driveL(double power) {
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        //runs while the ultasonic sensor reads higher than the desired cm
        while (opModeIsActive() && (!lineThere())) {
            drive(power);
            telemetry.clearAll();
            telemetry.addData("Driving", "now");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.addData("Line Color", line.getRawLightDetected());
            telemetry.update();
        }
        telemetry.addData("Drive", "Complete");
        telemetry.update();
        stopMotors();
    }

    //follows the line
    public void followLineRight(double power) {
        line.enableLed(true);
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        //runs while encoders give a position less than the intended distance
        while (!lineThere()) {
            pivotRight(0.2);
        }
        while (opModeIsActive()) {
            if (!lineThere()) {
                leftMotor.setPower(power + 0.01);
                rightMotor.setPower(power - 0.01);
            } else if (lineThere()) {
                leftMotor.setPower(power - 0.01);
                rightMotor.setPower(power + 0.01);
            }
            telemetry.clearAll();
            telemetry.addData("Following", lineThere());
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Following", "Complete");
        telemetry.update();
        stopMotors();

    }

    public void pivotRight(double radians, double power) {
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        while (opModeIsActive() && (rightMotor.getCurrentPosition() < MMtoTicks((int) (radians * 220)))) {
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            telemetry.clearAll();
            telemetry.addData("Pivoting", "Right");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Pivot Right", "Complete");
        telemetry.update();
        stopMotors();
    }

    //sets wheel power so we can endlessly turn
    public void pivotRight(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    public void pivotLeft(double radians, double power) {
        radians = calibrateTurn(radians);
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        while (opModeIsActive() && (leftMotor.getCurrentPosition() < MMtoTicks((int) (radians * 220)))) {
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            telemetry.clearAll();
            telemetry.addData("Pivoting", "Left");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Pivot Left", "Complete");
        telemetry.update();
        stopMotors();
    }

    //does sweeping turns
    public void turnLeft(double radius, double radians, double power) {
        resetEncoders();
        double outerCirc = Math.PI * radius;
        double innerCirc = Math.PI * (radius - 220);
        while (opModeIsActive() && (rightMotor.getCurrentPosition() < MMtoTicks((int) (radius * radians)))) {
            leftMotor.setPower(power);
            rightMotor.setPower(power * (innerCirc / outerCirc));
            telemetry.addData("Turning", "Left");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
        }
        stopMotors();
    }

    //sets wheel power so we can endlessly turn
    public void pivotLeft(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    //sets a certain power
    public void drive(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void stopMotors() {
        drive(0);
    }

    //checks for line precense
    public boolean lineThere() {
        telemetry.addData("Line There", line.getRawLightDetected());
        return line.getRawLightDetected() > 1.8;
    }

    //calibrates turing istance to account wheel slide
    public double calibrateTurn(double radians) {
        return radians * 0.95;
    }

    private void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDirection(String direction) {
        if (direction.equals("FORWARD")) {
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (direction.equals("REVERSE")) {
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
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

}
