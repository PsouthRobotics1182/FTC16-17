package org.firstinspires.ftc.teamcode; //This is the address within the thing to the left that this things folder is in

import com.qualcomm.robotcore.eventloop.opmode.OpMode;  //This imports things that we use in the program that are not
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;  // Already in it by default
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OldOpmodes.Drive;


@TeleOp(name = "TestBot", group = "TeleOp")  //This line tells the phone that we want to use this program as
public class TestBot extends OpMode {               //an op mode and what its name and group it will be in is
  //That last line initialized the class TestBot as an OpMode. An op mode is just a program the robot executes


    //drive motors
    DcMotor leftMotor;    //This creates an "object" called leftMotor. This basically is what we will be referring to a specific motor as from now on
    DcMotor rightMotor;   //This also creates an "object" called rightMotor. It is a different motor than the left motor but they both right now have the same values.

    //drive values
    double left; //You'll see what these mean later in the code. Basically they are making arbitrary numbers that we will assign specific values later.
    double right;
    double a;

    //scale for expo

    @Override
    public void init() {   //This is the process that the robot follows to "boot" up.

        //Mapping physical motors to variable
        leftMotor = hardwareMap.dcMotor.get("left_Motor");  //The stuff in the paranthesis is what a motor is mapped to in the config of the phone.
        rightMotor = hardwareMap.dcMotor.get("right_Motor"); // Ask Austin how that works because it would be kind of hard to comment in its function

        leftMotor.setDirection(DcMotor.Direction.FORWARD); //This is setting both of the motors directions. This is important because
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// They may be oriented differently on the chassis and they may have to swap.
    }

    @Override
    public void loop() {  //This is the loop the robot executes while the teleOp opmode is running.
        //getting drive values from controller
        left = gamepad1.left_stick_y;  //Remember those arbitrary numbers from earlier? This is assigning those a value that is
        right = gamepad1.right_stick_y;// Equal to the value of the gamepad stick's y position which is a value between 1 and -1.

        //clipping values so they dont exceed 100% on motors
        left = Range.clip(left, -1, 1); //This is making sure that the values are between 1 and -1 in case someone screwed up
        right = Range.clip(right, -1, 1);

        //scale drive values for easier controlling
        left = Drive.expo(left);
        right = Drive.expo(right);


        if (gamepad2.dpad_down)  //These control statements basically make the wheels spin faster or slower if the dPad button up
            a = 0.3; // or down is being pushed
        if (gamepad2.dpad_up)
            a = 1;

        //set drive values for
        left = left * a;              //Scaling the settings of those not so arbitrary numbers by the dPad setting
        right = right * a;
        leftMotor.setPower(left);     //Finally assigning those speeds to the motors.
        rightMotor.setPower(right);

        //telemetry
        telemetry.addData("Left Motor power", left);       //Returning telemetry values to the driver station.
		telemetry.addData("Right Motor power", right);
        telemetry.addData("Speed", a);
    }


}