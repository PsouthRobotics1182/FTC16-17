package org.firstinspires.ftc.teamcode.OldOpmodes;

import android.app.Activity;
import android.media.MediaPlayer;
import android.view.View;
import android.widget.MediaController;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
1.) Map hardware
2.) Set Motor Direction
3.) Calibrate Gyro
4.) Wait for time
5.) Drive
 */
public class Autonomous extends LinearOpMode{
    Gyro gyro = new Gyro();
    VideoStream stream = new VideoStream(((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout));
    public void runOpMode() throws InterruptedException{
        //Maybe Play a Sounda
        /*int toneLocal = R.raw.tone;
        final MediaPlayer tone = new MediaPlayer();
        tone.setDataSource(R.raw.tone);*/
        Drive drive = new Drive(gyro.getCalStraight());
        drive.mapHardware();
        drive.setDirection("FORWARDS");
        gyro.calibrate();
        waitForStart();
        stream.start();
        gyro.integrateGyro();
        drive.startTelemetry();
        drive.driveT(10, 0);
        drive.halt(2000);
        drive.driveD(10, 0);
        drive.halt(2000);
        drive.driveEncoders(36);
        drive.halt();
    }
}
