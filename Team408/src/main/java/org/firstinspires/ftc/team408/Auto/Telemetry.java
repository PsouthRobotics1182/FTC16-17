package org.firstinspires.ftc.team408.Auto;

/**
 * Created by Robotics on 2/14/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team408.Auto.AutoLib;


@TeleOp(name = "Telemetry", group = "TeleOp")
public class Telemetry extends AutoLib {

    public void runOpMode() throws InterruptedException
    {
        config();
        waitForStart();
        setEncoders();
        Telemetryloop();
    }

    public void Telemetryloop() {

        while (opModeIsActive()) {
            //Returns telemetry data to the driver phone
            telemetry.addData("Left Motor power: ", left);
            telemetry.addData("Right Motor power: ", right);
            telemetry.addData("Popper Motor power: ", popper);
            telemetry.addData("Left Motor Position:", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Position:", rightMotor.getCurrentPosition());
            telemetry.addData("Lift Position 1: ", lift.getCurrentPosition());
            telemetry.addData("Lift Position 2: ", lift2.getCurrentPosition());
            telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
            telemetry.addData("Lift Power: ", liftPower);
            telemetry.addData("Light 1: ", lightSensor.getRawLightDetected());
            telemetry.addData("Light 2: ", lightSensor2.getRawLightDetected());
            telemetry.addData("Sonar: ", rangeSensor.rawUltrasonic());
            telemetry.addData("Color Left RED", color.red());
            telemetry.addData("Color Left BLUE", color.blue());


            if ((color.blue() > color.red())) {
                telemetry.addData("Color Pick: ", "Blue");
            }
            else if (color.blue() < color.red()) {
                telemetry.addData("Color Pick: ", "Red");
            }
            else
            {
                telemetry.addData("Color Pick: ", "None");
            }

            telemetry.update();


        }
    }
}
