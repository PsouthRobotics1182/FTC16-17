/*
package org.firstinspires.ftc.teamcode.OldOpmodes;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.view.View;

public class VideoStream extends Activity {

    public void runOpMode(){}
    private static final int CAMERA_REQUEST = 1888;

    View relativeLayout;
    public VideoStream(View relativeLayout){
        this.relativeLayout = relativeLayout;
    }
    public void stream(){
        relativeLayout.post(new Runnable() {
            public void run() {
                Intent cameraIntent = new Intent(android.provider.MediaStore.ACTION_IMAGE_CAPTURE);
                startActivityForResult(cameraIntent, CAMERA_REQUEST);
            }
        });
    }


    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == CAMERA_REQUEST) {
            Bitmap photo = (Bitmap) data.getExtras().get("data");
            Drawable background = new BitmapDrawable(getResources(), photo);
            relativeLayout.setBackground(background);
        }

    }


    Runnable streamVideo = new Runnable() {
        public void run() {
            stream();
        }
    };
    public void start() {
        Thread videoStream = new Thread(streamVideo);
        videoStream.setDaemon(true);
        videoStream.start();
    }
}*/
