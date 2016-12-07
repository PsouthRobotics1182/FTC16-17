package org.firstinspires.ftc.team406.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.team406.R;

import java.util.Scanner;

/**
 * Created by dzogh_000 on 12/4/2016.
 */
@Autonomous
public class VuforiaNavigation extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        final String LICENSE_KEY = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";

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

        telemetry.addData("Vuforia", "Setup");
        telemetry.update();
        waitForStart();

        picturs.activate();

        while (opModeIsActive()){
            for (VuforiaTrackable picture: picturs){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)  picture.getListener()).getPose();

                if (pose != null){
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(picture.getName() + " Translation", translation);
                    //calculates angle based on sidelength                 1 if vertical 0 for horixontal

                    Scanner sc = new Scanner(pose.formatAsTransform()).useDelimiter(" ");
                    telemetry.addData(picture.getName() + " Transformation", pose.formatAsTransform());


                }
            }
            telemetry.update();
        }

        picturs.deactivate();
    }

    public double getAngle(OpenGLMatrix matrix){
        String matric = matrix.formatAsTransform();
        Scanner s = new Scanner(matric).useDelimiter(" ");

        s.next();
        s.next();
        s.next();
        double angle = s.nextDouble();
        return  angle;
    }

}
