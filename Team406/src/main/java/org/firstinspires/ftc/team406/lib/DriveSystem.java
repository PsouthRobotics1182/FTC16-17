package org.firstinspires.ftc.team406.lib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;

public abstract class DriveSystem extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    }

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    ModernRoboticsI2cRangeSensor range;

    LightSensor lineC;
    LightSensor lineF;

    //andymark motor specs
    static int ticksPerRevolutionAndy = 1120;
    static int maxRPMAndy = 129;
    static int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;
    static int ticksPerRevolutionAndy60 = 1680;
    static int maxRPMAndy60 = 105;
    static int maxTicksPerSecondAndy60 = maxRPMAndy60 * ticksPerRevolutionAndy60;
    //tertix motor specs
    static int ticksPerRevolutionTertix = 1440;
    static int maxRPMTetrix = 142;
    static int maxTicksPerSecondTetrix = maxRPMTetrix * ticksPerRevolutionTertix;

    public void configure() {

        telemetry.clearAll();
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        lineC = hardwareMap.lightSensor.get("lineC");
        lineF = hardwareMap.lightSensor.get("lineF");
        setDirection("FORWARD");

        //set run mode to use encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMaxSpeed(maxTicksPerSecondAndy);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy);
        //makes it so the motor cant be backdriven easily
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lineC.enableLed(true);
        lineF.enableLed(true);
    }

    //drives forward a certain distance
    public void drive(int mm, double power) throws InterruptedException {

        telemetry.clearAll();
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        //runs while encoders give a position less than the intended distance
        while (opModeIsActive() && (leftMotor.getCurrentPosition() < MMtoTicks(mm) && rightMotor.getCurrentPosition() < MMtoTicks(mm))) {

            double percentToGo = rightMotor.getCurrentPosition()/ MMtoTicks(mm);
            percentToGo += 0.1;
            if (percentToGo > 1) percentToGo = 1;
            drive(power);
            telemetry.clearAll();
            telemetry.addData("Driving", "now");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        telemetry.addData("Drive", "Complete");
        telemetry.update();
        stopMotors();
    }

    //drives until sonar reads a certain distance
    public void driveD(int mm, double power) throws InterruptedException {  //uses mm so it is consistnent with the rest of the methods

        telemetry.clearAll();
        telemetry.addData("Driving Distance", range.cmUltrasonic());
        telemetry.update();
        //converts mm to cm
        int cm = mm / 10;
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        //runs while the ultasonic sensor reads higher than the desired cm
        while (opModeIsActive() && (range.cmUltrasonic() > cm)) {
            drive(power);
            telemetry.clearAll();
            telemetry.addData("Driving Distance", range.cmUltrasonic());
            telemetry.update();
            idle();
        }

        if (range.cmUltrasonic() < 10){
            driveR((int) (cm-range.cmUltrasonic()), 0.3);
        }
        telemetry.addData("Drive", "Complete");
        telemetry.update();
        stopMotors();
    }

    //drives in reverse a certain distance
    public void driveR(int mm, double power) throws InterruptedException {

        telemetry.clearAll();
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
            idle();
        }
        setDirection("FORWARD");
        telemetry.addData("Drive", "Complete");
        telemetry.update();
        stopMotors();
    }

    //drives until it sees a line
    public void driveLF(double power) throws InterruptedException{
        telemetry.clearAll();
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        //runs while the ultasonic sensor reads higher than the desired cm
        while (opModeIsActive() && !lineThereFront()) {
            drive(power);
            telemetry.clearAll();
            telemetry.addData("Following", "now");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
//            telemetry.addData("Left Line Color", lineL.getRawLightDetected());
//            telemetry.addData("Right Line Color", lineR.getRawLightDetected());
            telemetry.addData("Range", range.cmUltrasonic());
            //telemetry.addData("Line", lineThere());
            telemetry.update();
            idle();
        }
        telemetry.addData("Follow", "Complete");
        telemetry.update();
        stopMotors();
    }
    public void driveLC(double power) throws InterruptedException {
        telemetry.clearAll();
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        //runs while the ultasonic sensor reads higher than the desired cm
        while (opModeIsActive() && !lineThereCenter()) {
            drive(power);
            telemetry.clearAll();
            telemetry.addData("Following", "now");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
//            telemetry.addData("Left Line Color", lineL.getRawLightDetected());
//            telemetry.addData("Right Line Color", lineR.getRawLightDetected());
            telemetry.addData("Range", range.cmUltrasonic());
            //telemetry.addData("Line", lineThere());
            telemetry.update();
            idle();
        }
        telemetry.addData("Follow", "Complete");
        telemetry.update();
        stopMotors();
    }

    public void pivotRight(double radians, double power) throws InterruptedException{

        telemetry.clearAll();
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        while (opModeIsActive() && (rightMotor.getCurrentPosition() < MMtoTicks((int) (radians * 220)))) {
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            telemetry.clearAll();
            telemetry.addData("Pivoting", "Right");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
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

    public void pivotLeft(double radians, double power) throws InterruptedException {

        telemetry.clearAll();
        radians = calibrateTurn(radians);
        resetEncoders(); //resets the encoders so i can keep a accurate distance measurements
        while (opModeIsActive() && (leftMotor.getCurrentPosition() < MMtoTicks((int) (radians * 220)))) {
            pivotLeft(power);
            telemetry.clearAll();
            telemetry.addData("Pivoting", "Left");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        telemetry.addData("Pivot Left", "Complete");
        telemetry.update();
        stopMotors();
    }

    //does sweeping turns
    public void turnLeft(double radius, double radians, double power) throws  InterruptedException {

        telemetry.clearAll();
        resetEncoders();
        double outerCirc = Math.PI * radius;
        double innerCirc = Math.PI * (radius - 220);
        while (opModeIsActive() && (rightMotor.getCurrentPosition() < MMtoTicks((int) (radius * radians)))) {
            leftMotor.setPower(power);
            rightMotor.setPower(power * (innerCirc / outerCirc));
            telemetry.addData("Turning", "Left");
            telemetry.addData("Left Motor Power", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Power", rightMotor.getCurrentPosition());
            idle();
        }
        stopMotors();
    }

    //sets wheel power so we can endlessly turn
    public void pivotLeft(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    //sets a certain power
    public void drive(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    //TODO when any command is run after this there is a variable delay lasting 3-10 sec
    public void stopMotors() {
        drive(0);
    }

    //checks for line precense
    public boolean lineThereCenter() {
        telemetry.addData("Line There", lineC.getLightDetected());
        return lineC.getLightDetected() > 0.5;
    }
    public boolean lineThereFront() {
        telemetry.addData("Line There", lineF.getLightDetected());
        return lineF.getLightDetected() > 0.4;
    }
    //public boolean lineThere(){        return lineThereLeft() || lineThereRight();}

    //calibrates turing istance to account wheel slidef
    public double calibrateTurn(double radians) {
        return radians * 0.90;
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
