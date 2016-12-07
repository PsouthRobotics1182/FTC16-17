package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.lang.Math;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by dzogh_000 on 9/17/2016.
 */
public class SensorSystem extends OpMode implements SensorEventListener {
    public void init(){}
    public void loop(){}

    GyroSensor gyro;
    public SensorSystem(){
        gyro = null;
    }

    public void useGyro(String name){
        gyro = hardwareMap.gyroSensor.get(name);
    }
    public double getRotation(){
        int heading = gyro.getHeading();
        return heading;
    }
    //variables for accelermoter
    private SensorManager mSensorManager;
    private Sensor accelerometer;
    double x;
    double y;
    double z;

    private float[] acceleration = {0.0f,0.0f,0.0f};    // SI units (m/s^2)
    private float[] mAccelerometer;

    public void useAcclerometer(){
        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        acceleration[0] = 0.0f;
        acceleration[1] = 0.0f;
        acceleration[2] = 0.0f;

        x = 5;
        y = 5;
        z = 5;
    }
    public void beginAccelerometer(){
        // delay value is SENSOR_DELAY_UI which is ok for telemetry, maybe not for actual robot use
        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
    }
    public float[] getAcceleration(){
        return acceleration;
    }
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // not sure if needed, placeholder just in case
    }
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            mAccelerometer = event.values;
            if (mAccelerometer != null) {
                acceleration[0] = mAccelerometer[0]; // Acceleration minus Gx on the x-axis
                acceleration[1] = mAccelerometer[1]; // Acceleration minus Gy on the y-axis
                acceleration[2] = mAccelerometer[2]; // Acceleration minus Gz on the z-axis
            }
        }
    }
    public void end(){
        mSensorManager.unregisterListener(this);
    }
}
