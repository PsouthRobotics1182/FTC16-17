package org.firstinspires.ftc.team408.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class designed to use a linear op mode for teleop
 * this allows us to extend linear opmode for extra features like sleeping
 * also allows me to use the same core function methods for tele-op and autonomous
 */

@TeleOp(name="Tele-Op")
public class teleOp extends LinearOpMode {
    //variables to set the sweeper and lift motor speeds, since they are constant during teleop
    private final double LIFT_POWER = 1;
    private final double SWEEPER_POWER = 1;

    boolean reverse = false;

    //AutoLibV2 robot;
    AutoLibV2 robot;

    public void runOpMode() throws InterruptedException {
        robot = new AutoLibV2(this, hardwareMap);

        //configure all hardware settings and layouts
        //robot = null;//new AutoLibV2(this);

        waitForStart();

        //control loop for driver control
        while (opModeIsActive()){

            //BBBBBB
            launchControl();//enables particle launching
            //soundEffects();//enables gamepad2 sound effects
            //updateVufoira();//retrieves position data
            //retrieves drive powers with speed and direction correction added
            double[] power = driveCharControl(gamepad1.right_stick_y, gamepad1.left_stick_y);
            //controls the cap ball lift
            //robot.cap.setPower(capControl());
            robot.cap.setPower(capControl());
            //sets all the mtor powers
            robot.leftMotor.setPower(power[0]);
            robot.rightMotor.setPower(power[1]);
            //these get the power from the methods which handle user input
            robot.liftMotor.setPower(liftControl());
            robot.sweeperMotor.setPower(sweepControl());
            if (gamepad2.left_bumper)
                robot.button.setPower(1);
            else if (gamepad2.left_trigger > 0.5)
                robot.button.setPower(-1);
            else
                robot.button.setPower(0);

            if (gamepad2.right_bumper)
                robot.button2.setPower(-1);
            else if (gamepad2.right_trigger > 0.5)
                robot.button2.setPower(1);
            else
                robot.button2.setPower(0);

            //telemetry.addData("Servo Positions", robot.cap.capHand.getPosition() + " / " + robot.cap.capHand2.getPosition());
            //telemetry.update();
            //showTelemetry();
        }

        robot.tts.stop();//stops the tts so we can reuse it
        robot.tts.shutdown();
    }
    //enables variable drive speed and reverse
    private double[] driveCharControl(double leftPower, double rightPower) {
        //array holding wheel powers

        double[] power = {leftPower, rightPower};//left then right
        if (gamepad2.dpad_up) {
            power[0] = 0.15;
            power[1] = 0.15;
        }
        else if (gamepad2.dpad_down) {
            power[0] = -0.15;
            power[1] = -0.15;
        }


        if (gamepad1.left_trigger > 0.5) {//button to slow the robot down
            power[0] = leftPower / 4;
            power[1] = rightPower / 4;
        } else if (gamepad1.right_trigger > 0.5 || reverse) {
            power[1] = -leftPower * 0.4;//option to reverse the robot
            power[0] = -rightPower * 0.4;
        }
        if (gamepad1.b)
            reverse = true;
        else if (gamepad1.a)
            reverse = false;
        if (reverse && robot.leftMotor.getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.BRAKE)) {
            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (!reverse && robot.leftMotor.getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.FLOAT)){
            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        return power;
    }
    //allows the gamepads to control the particle lift
    private double liftControl(){
        if (gamepad1.dpad_down)//button to runa at constant power
            return -LIFT_POWER;
        else if (gamepad1.dpad_up)//button to run down
            return LIFT_POWER;
            //allows gamepad 2 to use variable speed for the ball lift
        else if (Math.abs(gamepad2.right_stick_y) > 0.1)
            return gamepad2.right_stick_y;
        else
            return 0;//return zero so if there is not input the motor wont run
    }
    //allows control of the cap ball
    private double capControl(){
        if (gamepad2.y) {
            robot.cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad2.x) {
            robot.cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (gamepad2.b)
            robot.cap.grab(true);
        else if (gamepad2.a)
            robot.cap.grab(false);
        if (gamepad2.left_trigger > 0.5)
            return gamepad2.left_stick_y/2;
        else
            return gamepad2.left_stick_y;
    }
    //controls the ball intake at a constant speed using bumpers
    private double sweepControl(){
        if (gamepad1.right_bumper || gamepad2.dpad_right)
            return SWEEPER_POWER;
        else if (gamepad1.left_bumper || gamepad2.dpad_left)
            return -SWEEPER_POWER;
        else
            return 0;//return zero so if there is not input the motor wont run
    }
    //controls the button pusher for gamepad 1 and 2
    private double servoControl(){
        if (gamepad1.dpad_left || gamepad2.left_bumper)
            return AutoLibV2.LEFT;
        else if (gamepad1.dpad_right || gamepad2.right_bumper)
            return AutoLibV2.RIGHT;
        else
            return 0;//return zero so if there is not input the motor wont run
    }
    //launches the particle if the button is pressedx
    private void launchControl() throws InterruptedException {
        if (gamepad1.x)
            robot.shoot();
    }
    //allows gamepad 2 to add sound effects
   /*private void soundEffects(){
       if (gamepad2.b)
           robot.speak("OOPS");
       else if (gamepad2.x)
           robot.speak("Five Points");
       else if (gamepad2.y)
           robot.speak("Wiggle, Wiggle, Wiggle");
       else if (gamepad2.left_stick_button)
           robot.speak("Oh Yeah");
       else if (gamepad2.right_stick_button)
           robot.speak("Good Game");
       else if (gamepad2.dpad_down)
           robot.speak("Gracious Professionalism");
       else if (gamepad2.dpad_up)
           robot.speak("Four O Six");
       else if (gamepad2.dpad_left)
           robot.speak("Swiggity Swoogity");
       else if (gamepad2.dpad_right)
           robot.speak("I wasn't suppose to say that");
   }
*/
    //dumps all the robots sensor info for debugging purposes
    private void showTelemetry(){
        telemetry.addData("Left Motor Power", robot.leftMotor.getPower());
        telemetry.addData("Right Motor Power", robot.rightMotor.getPower());
        telemetry.addData("Particle Lift Motor Power", robot.liftMotor.getPower());
        telemetry.addData("Cap Ball Lift Motor Power", robot.cap.getPower());
        telemetry.addData("Sweeper Motor Power", robot.sweeperMotor.getPower());
        telemetry.addData("Button CRServo Power", robot.button.getPower());
        telemetry.addData("Range Sensor Distance", robot.range.cmUltrasonic());
        telemetry.addData("Front Light Sensor Brightness", robot.frontL.getLightDetected());
        telemetry.addData("Center Light Sensor Brightness", robot.centerL.getLightDetected());
        telemetry.addData("Gamepad 1 Bumper", gamepad1.right_bumper);
        telemetry.addData("Gamepad 2 A: ", gamepad2.a);
        telemetry.addData("Gamepad 2 B: ", gamepad2.b);
        telemetry.update();
    }


}
