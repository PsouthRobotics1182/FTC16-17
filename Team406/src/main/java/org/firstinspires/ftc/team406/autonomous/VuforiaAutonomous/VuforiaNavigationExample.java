package org.firstinspires.ftc.team406.autonomous.VuforiaAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.team406.R;
import org.firstinspires.ftc.team406.lib.DriveSystem;

//@Autonomous
public class VuforiaNavigationExample extends DriveSystem {

    @Override
    public void runOpMode() throws InterruptedException {

        final String LICENSE_KEY = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";

        telemetry.addData("Drive System", "Complete");
        telemetry.addData("Vuforia Setup", "In Progress");
        telemetry.update();

        //Sets all of the vuforia parameters
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //makes axes show on vuforia picture
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //Creates vuforia instance
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        //Allows vuforia to see 4 pictures
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        //Loads pre defined trackable files for beacon images into vuforia
        VuforiaTrackables picturs = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        //Gives each trackable a name
        picturs.get(0).setName("Wheels");
        picturs.get(1).setName("Tools");
        picturs.get(2).setName("Lego");
        picturs.get(3).setName("Gears");

        //creates listener for the wheels picture
        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) picturs.get(0).getListener();

        telemetry.addData("Vuforia Setup", "Complete");
        waitForStart();
        picturs.activate();
        //Drives forward until the picture is in view
        drive(0.2);
        while (wheels.getRawPose() == null) {
            drive(0.2);
        }

        stopMotors();

        sleep(1000);

        //TODO analyze beacon color

        //analyzes picture llocation to calculate nessacary movement
        while (wheels.getPose() == null){}
        VectorF angles = anglesFromTarget(wheels);
        VectorF translation = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));

        if (translation.get(0) > 0) //if x is positive beacon is to the right so turn
            pivotRight(0.2);
        else
            pivotLeft(0.2);//of turn left if it is negitive

        do {
            if (wheels.getPose() != null)
                //update translation while we pivot
                translation = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));
            idle();
        } while (opModeIsActive() && Math.abs(translation.get(0)) > 30);

        stopMotors();

        //Drives towards the beacon calculating distance with the two triangle side lengths
        drive((int) Math.hypot(translation.get(0), translation.get(2) + 215));

        //this corrects for any overshooting the robot may do
        while(opModeIsActive() && (wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10)){
            if (wheels.getPose() != null) {
                if (wheels.getPose().getTranslation().get(0) > 0)
                    pivotRight(0.2);
                else
                    pivotLeft(0.2);
            }else
                pivotRight(0.2);
        }
        stopMotors();

        picturs.deactivate();
    }

    private VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall) {
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    private VectorF anglesFromTarget(VuforiaTrackableDefaultListener image) {
        float[] data = image.getRawPose().getData();
        float[][] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float) thetaX, (float) thetaY, (float) thetaZ);
    }

}