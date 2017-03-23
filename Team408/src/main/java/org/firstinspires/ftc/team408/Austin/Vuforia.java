package org.firstinspires.ftc.team408.Austin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.team408.R;


/**
 * Created by Robotics on 2/22/2017.
 */

@Autonomous(name = "Vuforia", group = "LinearOpMode")
public class Vuforia extends LinearOpMode {

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target1;
    VuforiaTrackable target2;
    VuforiaTrackable target3;
    VuforiaTrackable target4;
    VuforiaTrackableDefaultListener listener;
    VuforiaTrackableDefaultListener listener2;
    VuforiaTrackableDefaultListener listener3;

    Boolean lineUp;
    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";

    public void runOpMode() throws InterruptedException{

        setupVuforia();

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        visionTargets.activate();

        while (opModeIsActive())
        {
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
            OpenGLMatrix latestLocation2 = listener2.getUpdatedRobotLocation();
            OpenGLMatrix latestLocation3 = listener3.getUpdatedRobotLocation();

            if (latestLocation != null)
                lastKnownLocation = latestLocation;

            if (latestLocation2 != null)
                lastKnownLocation = latestLocation2;

            if (latestLocation3 != null)
                lastKnownLocation = latestLocation3;

            telemetry.addData("Tracking " + target1.getName(), listener.isVisible());
            telemetry.addData("Tracking " + target2.getName(), listener2.isVisible());
            telemetry.addData("Tracking " + target3.getName(), listener3.isVisible());
            telemetry.addData("Last Known Location ", formatMatrix(lastKnownLocation));
            telemetry.addData("LastKnown Location ", lastKnownLocation);

            lineUp = false;

            OpenGLMatrix lineUpLocation = createMatrix(1447, 308, 15, 90, 1, -142);

            if (lastKnownLocation == lineUpLocation)
            {
                lineUp = true;
                telemetry.addData("LINED UP!!!!!!", "");
            }

            telemetry.addData("Lined Up for Auto: ", formatMatrix(lineUpLocation));


            telemetry.update();
            idle();
        }
    }

    public void setupVuforia() {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");

        target1 = visionTargets.get(0);
        target1.setName("Wheels Target");
        target1.setLocation(createMatrix(0, 1525, 0, 90, 0, 90));

        target2 = visionTargets.get(1);
        target2.setName("Tools Target");
        target2.setLocation(createMatrix(0, 2745, 0, 90, 0, 90));

        target3 = visionTargets.get(2);
        target3.setName("Legos Target");
        target3.setLocation(createMatrix(1525, 0, 0, 90, 0, 0));



        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        listener = (VuforiaTrackableDefaultListener) target1.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listener2 = (VuforiaTrackableDefaultListener) target2.getListener();
        listener2.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listener3 = (VuforiaTrackableDefaultListener) target3.getListener();
        listener3.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y , z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}
