/*
package org.firstinspires.ftc.teamcode.OldOpmodes;

import android.os.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

*/
/**
 * Created by dzogh_000 on 4/11/2016.
 * class to handle gyro calibration
 *//*

public class Gyro extends LinearOpMode {
    GyroSensor gyroC;
    GyroSensor gyroT;
    double calStraight;
    //variable to hold angle
    static double angle = 0;

    //method for calibration
    public Gyro() {
    }

    public void runOpMode() {
    }

    public void calibrate() throws InterruptedException {
        gyroC = hardwareMap.gyroSensor.get("gy");

        double sumValues = 0;
        //create array for holding sample values
        double[] gyroValues;
        gyroValues = new double[20];
        //collect the sample values
        //f defines how many samples sbhould be taken
        int f = 20;
        for (int i = 0; i < f; i++) {
            sleep(20);
            gyroValues[i] = gyroC.rawZ();
            //sum the sample values together
            sumValues += gyroValues[i];
        }
        calStraight = sumValues / f;
    }

    public double getCalStraight() {
        return calStraight;
    }

    public static double getAngle(){
        return angle;
    }

    Runnable gyroIntegrationRunnable = new Runnable() {
        public void run() {
            android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_BACKGROUND);
            while (opModeIsActive()) {
                gyroT = hardwareMap.gyroSensor.get("gy");

                angle += (gyroT.rawZ() / 100);
                telemetry.addData("Angle: ", angle);
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                }
            }

        }
    };

    public void integrateGyro() {
        Thread gyroIntegrator = new Thread(gyroIntegrationRunnable);
        gyroIntegrator.setDaemon(true);
        gyroIntegrator.start();
    }

}
*/
