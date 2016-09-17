package org.firstinspires.ftc.teamcode;

import android.app.Activity;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Streamtest extends LinearOpMode {
    VideoStream stream = new VideoStream(((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout));


    public void runOpMode() throws InterruptedException{


        waitForStart();
        stream.start();
        sleep(10000);
    }
}
