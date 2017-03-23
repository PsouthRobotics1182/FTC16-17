
package org.firstinspires.ftc.teamcode.VuforiaAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

@Autonomous(name = "AutoMoose")
public class VuforiaGears extends LinearOpMode {


    final String TAG = "Vuforia";
    final String LICENSE_KEY = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";
    //stores Vuforia instance
    VuforiaLocalizer vuforia;
    private OpenGLMatrix lastLocation;
    OpenGLMatrix trackablePose;

    List<VuforiaTrackable> allTrackables;

    VuforiaTrackables parts;


    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor liftMotor;
    DcMotor sweeperMotor;
    DcMotor launchMotor;

    GyroSensor gyro;

    //andymark motor specs
    int ticksPerRevolutionAndy = 1120;
    int maxRPMAndy = 129;
    int maxTicksPerSecondAndy = maxRPMAndy * ticksPerRevolutionAndy;
    //tertix motor specs
    int ticksPerRevolutionTertix = 1440;
    int maxRPMTetrix = 142;
    int maxTicksPerSecondTetrix = maxRPMTetrix * ticksPerRevolutionTertix;

    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        liftMotor = hardwareMap.dcMotor.get("liftM");
        sweeperMotor = hardwareMap.dcMotor.get("sweepM");

        launchMotor = hardwareMap.dcMotor.get("launchM");

        gyro = hardwareMap.gyroSensor.get("gyro");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);


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
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setup vuforia parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //add trackable objects to vuforia instance from assets folder
        parts = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable wheelsTarget = parts.get(0);
        wheelsTarget.setName("Wheels");

        VuforiaTrackable toolsTarget = parts.get(1);
        toolsTarget.setName("Tools");

        VuforiaTrackable legosTarget = parts.get(2);
        legosTarget.setName("Legos");

        VuforiaTrackable gearsTarget = parts.get(3);
        gearsTarget.setName("Gears");

        //add all trackables in a list for convenience
        allTrackables = new ArrayList<>();
        allTrackables.addAll(parts);

        //unit is mm
        float botWidth = (float) 457.2;
        float feildWidth = 3658;


/*to locate the robot based on the location of the trackable
        we must tell vuforia where the trackables are on the feild
        we do this by using transformations from the origin to tell vuforia where the objects are*/


        //place the wheels on the wall.
        OpenGLMatrix wheelsTargetLocation = OpenGLMatrix
                .translation(305, feildWidth / 2, 12)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        wheelsTarget.setLocation(wheelsTargetLocation);
        RobotLog.ii(TAG, "Wheel Target=%s", format(wheelsTargetLocation));


        //place the tools on the wall
        OpenGLMatrix toolsTargetLocation = OpenGLMatrix
                .translation(-feildWidth / 2, -762, 12)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYX,
                        AngleUnit.DEGREES, 90, 90, 0));
        toolsTarget.setLocation(toolsTargetLocation);
        RobotLog.ii(TAG, "Tool Target=%s", format(toolsTargetLocation));

        //place the legos on the wall
        OpenGLMatrix legosTargetLocation = OpenGLMatrix
                .translation(-762, feildWidth / 2, 12)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        legosTarget.setLocation(legosTargetLocation);
        RobotLog.ii(TAG, "Lego Target=%s", format(legosTargetLocation));


        //place the gears on the wall
        OpenGLMatrix gearsTargetLocation = OpenGLMatrix
                .translation(-feildWidth / 2, -305, 12)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gearsTarget.setLocation(gearsTargetLocation);
        RobotLog.ii(TAG, "Gear Target=%s", format(gearsTargetLocation));


        //make a another transformation matrix to describe where phone in on robot
        //this puts it portait on the right middle of the robot, 9cm above ground
        OpenGLMatrix phoneLocationObBot = OpenGLMatrix
                //TODO measure these values for testing and reenter
                .translation(0, botWidth, 90)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 0, -90));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationObBot));

        //add trackable listeners
        ((VuforiaTrackableDefaultListener) wheelsTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) toolsTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) legosTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) gearsTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);

        telemetry.addData("Vuforia", " Setup is Complete");
        telemetry.update();
        waitForStart();
        //begin tracking the images
        parts.activate();

        //resets encoders when you press start
        // leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //gyro.resetZAxisIntegrator();

        int[] values = getDistance();

        sleep(10000);
        if (values[1] > 0){
            pivotRight(degreesToRadians(Math.abs(90-values[1])));
            drive(Math.abs(values[3]));
            pivotLeft(Math.PI/2);
            values = getDistance();
            drive(Math.abs(values[4]));
        }
        if (values[1] < 0){
            pivotLeft(degreesToRadians(Math.abs(90-values[1])));
            drive(Math.abs(values[3]));
            pivotRight(Math.PI/2);
            values = getDistance();
            drive(Math.abs(values[4]));
        }

        telemetry.update();
        idle();


    }
    //small method to extract position information from a transformation matrix
    private String format(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }
    public double ticksToMM(int ticks){
        double revolutions = (double) ticks * ticksPerRevolutionAndy;

        double circ = 3.24459259 * 101.6;
        double mm = revolutions * circ;
        return mm;
    }
    public double MMtoTicks(int mm){
        double circ = 3.24459259 * 101.6;
        double revolutions = mm/circ;

        double ticks = revolutions * ticksPerRevolutionAndy;
        return ticks;
    }

    public void shoot(){

        //launchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int startPos = launchMotor.getCurrentPosition();
        //launchMotor.setTargetPosition((int) (startPos+(ticksPerRevolutionAndy* 0.5)));
        //launchMotor.setPower(.8);


        while (launchMotor.getCurrentPosition() < startPos + (ticksPerRevolutionAndy * 1.1)) {
            telemetry.addData("Shooting", "In Progress");
            telemetry.update();
            launchMotor.setPower(.4);
        }
        launchMotor.setPower(0);
        telemetry.addData("Shooting", "Complete");

    }
    public void drive(int mm){
        resetEncoders();
        while(leftMotor.getCurrentPosition() < MMtoTicks(mm)){
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
            telemetry.clearAll();
            telemetry.addData("Driving", "now");
            telemetry.addData("Left Motor Power", leftMotor.getPower());
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.update();
        }
        telemetry.addData("Drive", "Complete");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void pivotRight(double radians) {
        resetEncoders();
        while(leftMotor.getCurrentPosition() < MMtoTicks((int)((double) radians * 200))){
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
            telemetry.clearAll();
            telemetry.addData("Pivoting", "Left");
            telemetry.addData("Left Motor Power", leftMotor.getPower());
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.update();
        }
        telemetry.addData("Pivot Left", "Complete");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void pivotLeft(double radians) {
        resetEncoders();
        while(rightMotor.getCurrentPosition() < MMtoTicks((int)((double) radians * 200))){
            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
            telemetry.clearAll();
            telemetry.addData("Pivoting", "Left");
            telemetry.addData("Left Motor Power", leftMotor.getPower());
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.update();
        }
        telemetry.addData("Pivot Left", "Complete");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void resetEncoders(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double degreesToRadians(double degrees){
        double radians = degrees * (Math.PI/180);
        return radians;
    }
    public int[] getDistance(){
        OpenGLMatrix pose;
        while(true){
            for (VuforiaTrackable trackable : allTrackables) {
                if (trackable.getName().equals("Gears")) {
                    */
/**
                     * getUpdatedRobotLocation() will return null if no new information is available since
                     * the last time that call was made, or if the trackable is not currently visible.
                     * getRobotLocation() will return null if the trackable is not currently visible.
                     *//*

                    telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
                    trackablePose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

                }
            }
            telemetry.addData("Looking", "For Picture");
            telemetry.update();
            if (trackablePose != null) {
                String location = format(trackablePose);
                telemetry.addData(format(trackablePose), "");
                telemetry.update();
                Scanner sc = new Scanner(location).useDelimiter(" ");
                String partOne = sc.next();
                String partTwo = sc.next();
                String partThree = sc.next();
                int angleX = Integer.parseInt(partThree);
                String partFour = sc.next();
                int angleY = Integer.parseInt(partFour);
                String partFive = sc.next();
                partFive = partFive.replace(partFive.substring(partFive.length() - 1), "");
                int angleZ = Integer.parseInt(partFive);
                String partSix = sc.next();
                double partSeven = sc.nextDouble();
                String partEight = sc.next();
                double lengthY = partSeven;
                partEight = partEight.replace(partEight.substring(partEight.length() - 1), "");
                double lengthZ = Double.parseDouble(partEight);
                telemetry.addData("Angle X", angleX);
                telemetry.addData("Angle Y", angleY);
                telemetry.addData("Angle Z", angleZ);
                telemetry.addData("Length Y", lengthY);
                telemetry.addData("Length Z", lengthZ);
                telemetry.update();
                int[] values = new int[5];
                values[0] = angleX;
                values[1] = angleY;
                values[2] = angleZ;
                values[3] = (int) lengthY;
                values[4] = (int) lengthY;
                telemetry.addData("Angle X", values[0]);
                telemetry.addData("Angle Y", values[1]);
                telemetry.addData("Angle Z", values[2]);
                telemetry.addData("Length Y", values[3]);
                telemetry.addData("Length Z", values[4]);
                telemetry.update();
                return values;

            }

        }
    }
}*/
