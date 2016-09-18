package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class Vuforia extends LinearOpMode {


    String TAG = "Vuforia";
    //stores Vuforia instance
    VuforiaLocalizer vuforia;

    final String LICENSE_KEY = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";

    OpenGLMatrix lastPostition;

    @Override
    public void runOpMode() throws InterruptedException {

        //setup vuforia parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //add trackable objects to vuforia instance from assets folder
        VuforiaTrackables parts = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable wheelsTarget = parts.get(0);
        wheelsTarget.setName("Wheels");

        VuforiaTrackable toolsTarget = parts.get(1);
        toolsTarget.setName("Tools");

        VuforiaTrackable legosTarget = parts.get(2);
        legosTarget.setName("Legos");

        VuforiaTrackable gearsTarget = parts.get(3);
        gearsTarget.setName("Gears");

        //add all trackables in a list for convenience
        List<VuforiaTrackable> allTrackables = new ArrayList<>();
        allTrackables.addAll(parts);

        float botWidth = (float) 457.2;
        float feildWidth = 3580;

        /*to locate the robot based on the location of the trackable
        we must tell vuforia where the trackables are on the feild
        we do this by using transformations from the origin to tell vuforia where the objects are*/

        //place the wheels on the wall.
        OpenGLMatrix wheelsTargetLocation = OpenGLMatrix
                .translation(305, feildWidth / 2, 6)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        wheelsTarget.setLocation(wheelsTargetLocation);
        RobotLog.ii(TAG, "Wheel Target=%s", format(wheelsTargetLocation));


        //place the tools on the wall
        OpenGLMatrix toolsTargetLocation = OpenGLMatrix
                .translation(-feildWidth / 2, -762, 6)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYX,
                        AngleUnit.DEGREES, 90, 90, 0));
        toolsTarget.setLocation(toolsTargetLocation);
        RobotLog.ii(TAG, "Tool Target=%s", format(toolsTargetLocation));

        //place the legos on the wall
        OpenGLMatrix legosTargetLocation = OpenGLMatrix
                .translation(-762, feildWidth / 2, 6)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        legosTarget.setLocation(legosTargetLocation);
        RobotLog.ii(TAG, "Lego Target=%s", format(legosTargetLocation));


        //place the gears on the wall
        OpenGLMatrix gearsTargetLocation = OpenGLMatrix
                .translation(-feildWidth / 2, -305, 6)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gearsTarget.setLocation(gearsTargetLocation);
        RobotLog.ii(TAG, "Gear Target=%s", format(gearsTargetLocation));


        //make a another transformation matrix to describe where phone in on robot
        OpenGLMatrix phoneLocationObBot = OpenGLMatrix
                //TODO measure these values for testing and reenter
                .translation(botWidth / 2, 0, 9)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationObBot));

        //add trackable listeners
        ((VuforiaTrackableDefaultListener) wheelsTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) toolsTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) legosTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) gearsTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);

        telemetry.addData("Vuforia", " Setup is Complete");
        telemetry.update();
        waitForStart();

        //begin tracking the images
        parts.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");
                //prints out position of trackable... hopefully
                //telemetry.addData(trackable.getName(), format(((VuforiaTrackableDefaultListener)trackable.getListener()).getPose()));
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null)
                    lastPostition = robotLocationTransform;
            }
            if (lastPostition != null) {
                telemetry.addData("Position", format(lastPostition));
            } else {
                telemetry.addData("Position", "Unknown");
            }
            telemetry.update();
            idle();
        }
    }

    //small method to extract position information from a transformation matrix
    String format(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }
}
