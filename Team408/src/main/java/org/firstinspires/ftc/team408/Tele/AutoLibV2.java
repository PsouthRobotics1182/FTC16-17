package org.firstinspires.ftc.team408.Tele;

import android.speech.tts.TextToSpeech;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Locale;

/**
 * This class is used as a general library
 * Used to setup the hardware
 * It contains useful methods for controlling the robot
 * All time units are msec
 * All distance units are MM
 * ALl power units are decimal percentages ex: 78% is 0.78
 * (Unless otherwise specified)
 */

public class AutoLibV2 {
    //constants so i dont have to remember stuff just names
    public static final int LEFT = -1;
    public static final int RIGHT = 1;
    public static final int BLUE = 5;
    public static final int RED = -5;
    //constants used to control motor speed for a 40 gear ratio motor
    public final static int ticksPerRevolutionAndy40 = 1120;
    public final static int maxRPMAndy40 = 129;
    public final static int maxTicksPerSecondAndy40 = maxRPMAndy40 * ticksPerRevolutionAndy40;
    public final static int ticksPerRevolutionAndy20 = 560;
    public final static int maxRPMAndy20 = 275;
    public final static int maxTicksPerSecondAndy20 = maxRPMAndy20 * ticksPerRevolutionAndy20;

    public final static int SHOOTING_DISTANCE = 700;
    //global variables for each hardware device
    //motors
    public DcMotor leftMotor;//left drive train motor, 20 andymark, encoder
    public DcMotor rightMotor;//right drive train motor, 20 andymark, encoder
    public DcMotor launchMotor;//the shooting motors, 60 andymark, encoder
    public DcMotor liftMotor;//the particle lift motor, 40 andymark, no encoder
    public DcMotor sweeperMotor;//the foam spinner, 40 andymark, no encoder

    //servos
    public CRServo button;//the servo used to move the beacon presser
    public CRServo button2;
    //sensors
    public LightSensor frontL;//the lego light sensor at the front of the robot
    public LightSensor centerL;//the lego light sensor at turning center of robot
    public ColorSensor color;//the color sensor above the button presser, looks at left of beacon
    //public ModernRoboticsI2cRangeSensor rangeL;//the ultrasonic range sensor on front bumper
    //public ModernRoboticsI2cRangeSensor rangeR;//the ultrasonic range sensor on front bumper
    public ModernRoboticsI2cRangeSensor range;
    public TextToSpeech tts;//used to vocalize what the robot is doing


    private LinearOpMode opMode;

    public CapMechanism cap;

    //method to initialize all the motors and sensors
    public AutoLibV2 (LinearOpMode opMode, HardwareMap hwMap) {
        this.opMode = opMode;
        cap = new CapMechanism(opMode, hwMap);
       /*connects sensors and motor objects to the physical devices*/
        //motors
        leftMotor = hwMap.dcMotor.get("rightM");
        rightMotor = hwMap.dcMotor.get("leftM");
        launchMotor = hwMap.dcMotor.get("launchM");
        liftMotor = hwMap.dcMotor.get("liftM");
        sweeperMotor = hwMap.dcMotor.get("sweepM");
        //servos
        button = hwMap.crservo.get("serv");
        button2 = hwMap.crservo.get("serv2");
        //sensors
        frontL = hwMap.lightSensor.get("lineF");
        centerL = hwMap.lightSensor.get("lineC");
        color = hwMap.colorSensor.get("color");
        //rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        //rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        range = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        //makes it so the wheels resist turning at zero power, or so they can be spun
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //float-always surgical tubing to spin, this avoids stressing the motor
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //sets the motor directions
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sweeperMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //uses the encoders to run using rpm rather than power
        resetEncoders();

        //sets the max speed so motor.setPower() uses a percentage of this speed rather than power
        leftMotor.setMaxSpeed(maxTicksPerSecondAndy20);
        rightMotor.setMaxSpeed(maxTicksPerSecondAndy20);
        launchMotor.setMaxSpeed(1680 * 105);//different since it is a 60 gear ratio motor
        //turns on LED for better line recognition
        frontL.enableLed(true);
        centerL.enableLed(true);
        //turns off color sensor LED since it makes it always read blue
        color.enableLed(false);

        //initialize the TTS
        tts = new TextToSpeech(FtcRobotControllerActivity.myContext, new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
                if (status != TextToSpeech.ERROR) {
                    tts.setLanguage(Locale.UK);
                }
            }
        });

        speak("Setup Complete");
        cap.grab(true);
    }
    /*method to navigate the center sensor directly onto the line
    the robot initially drives quickly to save time, it then sees the line
    with the first sensor and slows down.  At the slower speed we look for the
    line using the center sensor*/
    //improves speed without sacrificing precision
    public void driveToLine() throws InterruptedException {
        //used to slowly spin up the motor power
        //prevents robot from pivoting slightly when it starts
        double windUp = 0.1;
        //run while the front sensor doesnt see the line
        while (!lineThere(frontL, 0.38) & opMode.opModeIsActive()) {
            //power is multiplied by windup which grows with each loop
            leftMotor.setPower(0.7 * windUp);
            rightMotor.setPower(0.7 * windUp);
            if (windUp < 1) //to prevent power from going over 1
                windUp += 0.01;
            opMode.idle();
        }
        //stops motor in between operations so there movement is precise as possible
        //added since the robot was having trouble transitioning smoothly
        stopMotors();
        //drive while the center light sensor does not see the line
        //a different threshold is used to detect the line to give a different location on the line
        //a higher value means farther past edge, smaller is closer to initial edge
        while (!lineThere(centerL, 0.46) && opMode.opModeIsActive()) {
            leftMotor.setPower(0.1);//drives really slow for precision
            rightMotor.setPower(0.1);
            opMode.idle();
        }
        stopMotors();
    }

    //drives at a power until the sonar reaches a certain distance
    //ability to fo back to reach distance as well
    //mm is used as parameter since all other methods use MM
    public void driveD(double power, int mm) throws InterruptedException {
        double cm = mm / 10;//convert the MM to CM
        if (range.cmUltrasonic() > cm) {//If we are too far from the wall
            while (opMode.opModeIsActive() && range.cmUltrasonic() > cm) {//drives until the sonar is less than the desired distance
                leftMotor.setPower(power);//drives forward
                rightMotor.setPower(power);
                opMode.telemetry.addData("Range", range.cmUltrasonic());
                opMode.telemetry.update();
                opMode.idle();
            }
        } else if (range.cmUltrasonic() < cm) {//if we are too close//drives until sonar is above desired distance
            while (opMode.opModeIsActive() && range.cmUltrasonic() < cm) {
                leftMotor.setPower(-power);//drives in reverse
                rightMotor.setPower(-power);
                opMode.telemetry.addData("Range", range.cmUltrasonic());
                opMode.telemetry.update();
                opMode.idle();
            }
        }
        opMode.telemetry.clearAll();
        stopMotors();
    }

    //drives a certain time at a certain power
    public void driveT(int time, double power) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        double windUp = 0.1;
        while(startTime + (long) time > System.currentTimeMillis() && opMode.opModeIsActive()){
            if (windUp < 1)windUp+=0.1;
            leftMotor.setPower(power*windUp);
            rightMotor.setPower(power*windUp);
        }
        stopMotors();
    }

    //drives for a certain distance using the encoders at a certain power
    //ability to drive in reverse by making the power parameter negative
    public void drive(int mm, double power) {
        int target = MMtoTicks(mm);//the target tick value calculated based on desired mm
        resetEncoders();//reset encoders so the values start at zero
        if (power > 0) {//if power is positive, go forward
            //drive until either wheel reaches its destination by comparing its encoder value to the calculated target
            while (opMode.opModeIsActive() && (leftMotor.getCurrentPosition() < target || rightMotor.getCurrentPosition() < target)) {
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }
        } else {//if power is negitive go in reverse
            //drive until either wheel reaches its destination by comparing its encoder value to the calculated target
            while (opMode.opModeIsActive() && (leftMotor.getCurrentPosition() > -target || rightMotor.getCurrentPosition() > -target)) {
                leftMotor.setPower(power);//power does not need to be reversed since it is already negative
                rightMotor.setPower(power);
            }
        }
        stopMotors();
    }

    //this pivots till the front sensor sees the line
    //it has three steps
    //1-go to line
    //2-go past
    //3 return to line
    //this is used so for both turning and driving to the line
    // so the light sensors approach the line from the same direction
    public void lineUp(int color) throws InterruptedException {
        double power = 0.2;//a low power is used to maintain precision
        if (color == RED) //flip the power for red to turn the opposite direction
            power = -power;
        //Step 1
        while (!lineThere(frontL, 0.46) && opMode.opModeIsActive()) {
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            opMode.idle();
        }
        //Step 2
        double threshhold;
        while (lineThere(frontL) && opMode.opModeIsActive()) {
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            opMode.idle();
        }
        //this is used so the motors will switch direction smoothly
        stopMotors();
        //Step 3
        while (!lineThere(frontL, 0.46) && opMode.opModeIsActive()) {
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            opMode.idle();
        }
        stopMotors();
    }

    //method to pivot a certain number number of radians
    public void pivot(double radians, double power) throws InterruptedException {
        //calculates ticks for the motors to reach
        //uses the arc length formula with radius as the robots radius,
        // this give MM then that is converted to encoder ticks
        int target = MMtoTicks((int) (radians * 205));//will be negative if radians is negative
        resetEncoders();
        //if turning right
        if (radians > 0) {
            //drives wheels while left encoders is less than target
            // since they will be going forward unlike the right wheels
            while (opMode.opModeIsActive() && leftMotor.getCurrentPosition() < target) {
                leftMotor.setPower(power);
                rightMotor.setPower(-power);
                opMode.idle();
            }
        }
        //if turning left
        else if (radians < 0) {
            //left encoder is used again since the target is negative
            //so the robot goes until the encoder reaches the negitive value
            //encoder value should be decreasing since the left wheel is going backward
            while (opMode.opModeIsActive() && leftMotor.getCurrentPosition() > target) {
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
                opMode.idle();
            }
        }
        //stop motors so i do not have to worry about it after calling a method
        stopMotors();
    }

    //method to stop both motors for ease of use
    public void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    //shoots the particle
    public void shoot() throws InterruptedException {
        opMode.telemetry.clearAll();
        //chose not to reset encoder since it is easier to just add rotation, unlike wheels
        int startPos = launchMotor.getCurrentPosition();

        //1680 is the ticks per revolution of the motors, multiplied by 1.1 so the motor doesnt stop short which could damage the gear teath
        while (opMode.opModeIsActive() && launchMotor.getCurrentPosition() < startPos + (1680 * 1.1)) {//drives while the launch motor hasnt reached one turn
            launchMotor.setPower(1);
            opMode.idle();
        }
        launchMotor.setPower(0);
    }
    //method to reload the shooter
    public void reload() throws InterruptedException {
        liftMotor.setPower(1);
        opMode.sleep(1500);
        liftMotor.setPower(0);
    }

    //takes a color as a parameter and presses the proper button
    public void pressButton(int target) throws InterruptedException {
       /*if (target == RED && whatColor() == RED)//if the left is red and we want red therefore,
           moveButton(LEFT);//we want to press the button on sensor side which is the left
       else if (target == RED && whatColor() == BLUE)//if the left is red but we want blue therefore,
           moveButton(RIGHT);//we want the other button, which is right
       else if (target == BLUE && whatColor() == BLUE)//if the left is blue and we want blue therefore,
           moveButton(LEFT);//we want eh sensor side button which is left
       else if (target == BLUE && whatColor() == RED)//if the left is blue but we want red therefore,
           moveButton(RIGHT);//we want the other side button which is right
*/
        driveD(0.7, 200);
        //drive for a certain time
        driveT(500, 0.2);//time is used because if we hit the wall the wheels wont spin, so encoders would get stuck
        stopMotors();
        drive(170, -0.4);//drives in reverse a known distance from he wall
    }

    //moves button to a certain position:Left or Right
    //dir is the LEFT or RIGHT constant which represetns power to go that direction
    private void moveButton(int dir) throws InterruptedException {
        button.setPower(dir);//sets the proper direction
        opMode.sleep(2000);// in 2 sec we know the servo will have completed its move
        button.setPower(0);
    }

    //returns what color the beacon is using the color sensor
    private int whatColor() {
        int red = color.red();
        int blue = color.blue();
        if (red > blue) return RED;//RED and BLUE are constants chosen to represent the colors
        else if (blue > red) return BLUE;
        else return 7;//used to show no color was decided
    }

    //returns whether the line is there or not
    private boolean lineThere(LightSensor light) {
        return light.getLightDetected() > 0.4;
    }

    //returns whether the line is there or not(same as above) but takes a threshold for the line
    private boolean lineThere(LightSensor light, double threshold) {
        return light.getLightDetected() > threshold;
    }

    //coverts encoder ticks count to mm using the radius of the wheel
    private int ticksToMM(int ticks) {
        double revolutions = (double) ticks * ticksPerRevolutionAndy40;

        double circ = 3.24459259 * 101.6;
        double mm = revolutions * circ;
        return (int) mm;
    }

    //converts mm to encoder ticks using wheel radius
    private int MMtoTicks(int mm) {
        double circ = 3.24459259 * 101.6;
        double revolutions = mm / circ;

        double ticks = revolutions * ticksPerRevolutionAndy40;
        return (int) ticks;
    }

    //resets all the encoder values of the drive train to zero
    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //method to interface with the TTS an dsay stuff
    public void speak(String string){
        tts.speak(string, TextToSpeech.QUEUE_FLUSH, null);
    }
}
