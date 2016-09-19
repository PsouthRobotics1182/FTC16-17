package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous
// Comment this out to add to the opmode list
public class Stream extends OpMode {

    View relativeLayout;

    @Override
    public void init() {

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        Intent cameraIntent = new Intent(android.provider.MediaStore.ACTION_IMAGE_CAPTURE);
        //startActivityForResult(cameraIntent, 1888);
    }

    @Override
    public void loop(){
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.CYAN);
            }
        });
    }


}
