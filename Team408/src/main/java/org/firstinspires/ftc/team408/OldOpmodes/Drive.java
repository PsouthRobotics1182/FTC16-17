package org.firstinspires.ftc.teamcode.OldOpmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;


public class Drive extends LinearOpMode {

    //Setup Variables for specific robot
    double wheelCircumfrence = 12;
    int numberOfDriveWheels = 4;


    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightRearMotor;

    GyroSensor gyro;

    UltrasonicSensor ultrasonicSensor;

    double calStraight;

    //to pass in calibrated straight value
    public Drive(double calStraight) {
        this.calStraight = calStraight;
    }

    public static double expo(double number) {
        double scaleNumber;
        if (number > 0)
            scaleNumber = 1.0672 / (1 + Math.pow(Math.E, (-9 * (number - 0.7))));
        else
            scaleNumber = 1.0672 / (1 + Math.pow(Math.E, (-9 * (-(number) - 0.7))));
        if (scaleNumber > 1)
            scaleNumber = 1;
        if (scaleNumber < -1)
            scaleNumber = -1;
        return scaleNumber;
    }

    //To satisfy interface
    public void runOpMode() {
    }

    public void halt(long milliseconds) throws InterruptedException {
        DbgLog.msg("Sleep Beginning");
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        sleep(milliseconds);
        DbgLog.msg("Sleep Complete");
    }

    public void halt() throws InterruptedException {
        DbgLog.msg("Sleep Beginning");
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        DbgLog.msg("Sleep Complete");
    }

    public void mapHardware() {
        try {
            leftFrontMotor = hardwareMap.dcMotor.get("LF");
        } catch (Exception e) {
            leftFrontMotor = null;
            telemetry.addData("Left Front Motor is NOT connected", "");
        }
        try {
            rightFrontMotor = hardwareMap.dcMotor.get("RF");
        } catch (Exception e) {
            rightFrontMotor = null;
            telemetry.addData("Right Front Motor is NOT connected", "");
        }
        if (numberOfDriveWheels == 4) {

            try {
                leftRearMotor = hardwareMap.dcMotor.get("LR");
            } catch (Exception e) {
                leftRearMotor = null;
                telemetry.addData("Left Rear Motor is NOT connected", "");
            }
            try {
                rightRearMotor = hardwareMap.dcMotor.get("RR");
            } catch (Exception e) {
                rightRearMotor = null;
                telemetry.addData("Right Rear Motor is NOT connected", "");
            }
        } else {
            leftRearMotor = null;
            rightRearMotor = null;
        }
        try {
            ultrasonicSensor = hardwareMap.ultrasonicSensor.get("ultraS");
        } catch (Exception e) {
            ultrasonicSensor = null;
            telemetry.addData("Ultra Sonic Sensor is NOT connected", "");
        }
        try {
            gyro = hardwareMap.gyroSensor.get("gy");
        } catch (Exception e) {
            gyro = null;
            telemetry.addData("Gyro is NOT connected", "");
        }


    }

    public void setDirection(String direction) {
        if (direction.equals("FORWARD")) {
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
            leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            DbgLog.msg("Set Direction Forward");
        }
        if (direction.equals("BACKWARDS")) {
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
            leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            DbgLog.msg("Set Direction Backward");
        }
    }

    public void driveT(double duration, double turn) throws InterruptedException {
        //coefficients for PID loop
        //                     P        I    D
        double[] pidValues = {0.0025, 0.0017, 3};
        DbgLog.msg("Proportional " + pidValues[0] + "\nIntegral " + pidValues[1] + "\nDerivative " + pidValues[2]);
        //Calibrate gyro for straight values
        //below is left above is right
        double straight = calStraight + turn;
        //setting drive speed
        double lSpeed = 0.6;
        double rSpeed = lSpeed;
        //How long to sleep between loop  cycles to help with math
        long dt = 20;
        //start time to compare against so loop only runs for so long
        double start_time = System.currentTimeMillis();
        //setting variables to zero to use in first loop round to avoid NULL errors
        double integral = 0;
        double previous_error = 0;
        //timer to run loop for given amount of time
        DbgLog.msg("Began While Loop for Driving");
        while (System.currentTimeMillis() - start_time < duration) {
            //error is how far off a straight value is to use for calculating corrections
            double error = straight - gyro.rawZ();
            //dividing error because motor speed is in percentage
            error = error / 1000;
            DbgLog.msg("Calculated Error");
            //proportional factor so correction is relative size of error
            double proportional = error;
            //integral helps deal with drift by calculating error over time and builds as the loop goes to deal with uncorrected error
            integral = integral + error * dt;
            DbgLog.msg("Calculated Integral");
            //derivative which uses slope of the error to correct future error and to prevent overshooting
            double derivative = (error - previous_error) / dt;
            DbgLog.msg("Calculated Derivative");
            //summing together to create complete correction value and multiplying by coefficients
            double PID = pidValues[0] * proportional + pidValues[1] * integral + pidValues[2] * derivative;
            DbgLog.msg("Calculated PID correction");
            //applying corrections to driving so we go straight
            lSpeed = lSpeed + PID;
            rSpeed = rSpeed - PID;
            lSpeed = Range.clip(lSpeed, 0, 1);
            rSpeed = Range.clip(rSpeed, 0, 1);
            leftFrontMotor.setPower(lSpeed);
            rightFrontMotor.setPower(rSpeed);
            leftRearMotor.setPower(lSpeed);
            rightRearMotor.setPower(rSpeed);
            DbgLog.msg("Applied Correction");
            //setting values for next loop so integral roles over
            previous_error = error;
            //telemetry stuff
            sendTelemetry();
            sleep(dt);
        }
        DbgLog.msg("Finished While Loop for Driving");

    }

    public void driveD(double cm, double turn) throws InterruptedException {
        //setting straight value
        //below is left above is right
        double straight = calStraight + turn;
        //setting drive speed
        double lSpeed = 0.6;
        double rSpeed = lSpeed;
        //How long to sleep between loop  cycles to help with math
        long dt = 20;
        //variables appended with G are for gyro control loop
        //variables appended with D are for the distance sensor

        //coefficients for PID loop
        //                      P        I    D
        double[] pidValuesG = {0.0025, 0.0017, 3};
        double[] pidValuesD = {0.7, 1, 1};
        DbgLog.msg("Proportional " + pidValuesG[0] + "\nIntegral " + pidValuesG[1] + "\nDerivative " + pidValuesG[2]);
        //setting variables to zero to use in first loop round to avoid NULL errors
        double integralG = 0;
        double integralD = 0;
        double previous_errorG = 0;
        double previous_errorD = 0;
        //Distance comparer so loop runs until the sensor reads a certain value
        DbgLog.msg("Began While Loop for Driving");
        while (ultrasonicSensor.getUltrasonicLevel() < cm) {

            //error is how far off a straight value is to use for calculating corrections
            double errorG = straight - gyro.rawZ();
            //error for distance in front for calculating speed.
            double errorD = cm - ultrasonicSensor.getUltrasonicLevel();
            //dividing error because motor speed is in percentage
            errorG = errorG / 1000;
            errorD = errorD / 2550;
            DbgLog.msg("Calculated Error");
            //proportional factor so correction is relative size of error
            double proportionalG = errorG;
            double proportionalD = errorD;
            //integral helps deal with drift by calculating error over time and builds as the loop goes to deal with uncorrected error
            integralG = integralG + errorG * dt;
            integralD = integralD + errorD * dt;
            DbgLog.msg("Calculated Integral");
            //derivative which uses slope of the error to correct future error and to prevent overshooting
            double derivativeG = (errorG - previous_errorG) / dt;
            double derivativeD = (errorD - previous_errorD) / dt;
            DbgLog.msg("Calculated Derivative");
            //summing together to create complete correction value and multiplying by coefficients
            double PIDg = pidValuesG[0] * proportionalG + pidValuesG[1] * integralG + pidValuesG[2] * derivativeG;
            double PIDd = pidValuesD[0] * proportionalD + pidValuesD[1] * integralD + pidValuesD[2] * derivativeD;
            DbgLog.msg("Calculated PID correction");
            //applying corrections to driving so we go straight
            lSpeed = lSpeed + PIDg + PIDd;
            rSpeed = rSpeed - PIDg + PIDd;
            lSpeed = Range.clip(lSpeed, -1, 1);
            rSpeed = Range.clip(rSpeed, -1, 1);
            leftFrontMotor.setPower(lSpeed);
            rightFrontMotor.setPower(rSpeed);
            leftRearMotor.setPower(lSpeed);
            rightRearMotor.setPower(rSpeed);
            DbgLog.msg("Applied Correction");

            //setting values for next loop so integral roles over
            previous_errorG = errorG;
            previous_errorD = errorD;
            sendTelemetry();

            sleep(dt);
        }
        DbgLog.msg("Finished While Loop for Driving");

    }

    //TODO use encoders to go distances and pivot
    public void driveEncoders(double inches) throws InterruptedException {
        double targetTicks = inchesToTicks(inches);
        //coefficients for PID loop
        //                     P        I    D
        double[] pidValues = {0.0025, 0.0017, 3};
        double[] ePidValues = {0.0025, 0.0017, 3};
        DbgLog.msg("Proportional " + pidValues[0] + "\nIntegral " + pidValues[1] + "\nDerivative " + pidValues[2]);
        //Calibrate gyro for straight values
        //below is left above is right
        double straight = calStraight;
        //setting drive speed
        double lSpeed = 0.6;
        double rSpeed = lSpeed;
        //How long to sleep between loop  cycles to help with math
        long dt = 20;
        //setting variables to zero to use in first loop round to avoid NULL errors
        double integral = 0;
        double encoderInt = 0;
        double previousError = 0;
        double encoderPrevError = 0;

        double startPosition = leftFrontMotor.getCurrentPosition();
        //timer to run loop for given amount of time
        DbgLog.msg("Began While Loop for Driving");
        while (leftFrontMotor.getCurrentPosition() - startPosition < targetTicks) {

            //error is how far off a straight value is to use for calculating corrections
            double encoderError = leftFrontMotor.getCurrentPosition() - rightFrontMotor.getCurrentPosition();
            double error = straight - gyro.rawZ();
            //dividing error because motor speed is in percentage
            error = error / 1000;
            DbgLog.msg("Calculated Error");
            //proportional factor so correction is relative size of error
            double proportional = error;
            double encoderPro = encoderError;
            //integral helps deal with drift by calculating error over time and builds as the loop goes to deal with uncorrected error
            integral = integral + error * dt;
            encoderInt = encoderInt + encoderError * dt;
            DbgLog.msg("Calculated Integral");
            //derivative which uses slope of the error to correct future error and to prevent overshooting
            double derivative = (error - previousError) / dt;
            double encoderDer = (encoderError - encoderPrevError) / dt;
            DbgLog.msg("Calculated Derivative");
            //summing together to create complete correction value and multiplying by coefficients
            double PID = pidValues[0] * proportional + pidValues[1] * integral + pidValues[2] * derivative;
            double ePID = ePidValues[0] * encoderPro + ePidValues[1] * encoderInt + ePidValues[2] * encoderDer;
            DbgLog.msg("Calculated PID correction");
            //applying corrections to driving so we go straight
            lSpeed = lSpeed + PID - ePID;
            rSpeed = rSpeed - PID + ePID;
            lSpeed = Range.clip(lSpeed, 0, 1);
            rSpeed = Range.clip(rSpeed, 0, 1);
            leftFrontMotor.setPower(lSpeed);
            rightFrontMotor.setPower(rSpeed);
            leftRearMotor.setPower(lSpeed);
            rightRearMotor.setPower(rSpeed);
            DbgLog.msg("Applied Correction");
            //setting values for next loop so integral roles over
            previousError = error;
            sendTelemetry();
            sleep(dt);
            waitOneFullHardwareCycle();
        }
        DbgLog.msg("Finished While Loop for Driving");

    }

    public double inchesToTicks(double inches) {
        double revolutions = inches / wheelCircumfrence;
        double ticks = revolutions * 1440;
        return ticks;
    }

    private void sendTelemetry() {
        telemetry.addData("Left Drive: ", leftFrontMotor.getPower());
        telemetry.addData("Right Drive: ", rightFrontMotor.getPower());
        telemetry.addData("Rotation: ", gyro.rawZ());
        telemetry.addData("Distance in Front: ", ultrasonicSensor.getUltrasonicLevel());
        telemetry.addData("Angle: ", Gyro.getAngle());
    }
    Runnable myTelemetry = new Runnable() {
        public void run() {
        sendTelemetry();
        }
    };
    public void startTelemetry(){
        Thread sendMyTelemetry = new Thread(myTelemetry);
        sendMyTelemetry.setDaemon(true);
        sendMyTelemetry.start();
    }


    //TODO see if this works if so make a teleop using encoders
    public void driveE(int inches) throws InterruptedException {
        DbgLog.msg("-------------------Encoder Drive Stream---------------");
        mapHardware();
        //reversing motors as needed
        setDirection("FORWARD");
        int revolutions = inches / 12;
        int targetTicks = revolutions * 1440;
        leftFrontMotor.setTargetPosition(targetTicks);
        DbgLog.msg("Changing left motor mode to run to position");
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DbgLog.msg("Mode Change Complete @target position MAYBE");

    }

}


