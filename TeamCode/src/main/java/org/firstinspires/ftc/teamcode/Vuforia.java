package org.firstinspires.ftc.teamcode;

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

public class Vuforia extends LinearOpMode {


    String TAG = "Vuforia";
    //stores Vuforia instance
    VuforiaLocalizer vuforia;

    final String LICENSE_KEY = "42ff9cnue02fn329ofn109fn8932";

    OpenGLMatrix lastPostition;

    @Override
    public void runOpMode() throws InterruptedException{

        //setup vuforia parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //add trackable objects to fuforia instance from assets folder
        VuforiaTrackables images = this.vuforia.loadTrackablesFromAsset("StonesAndChips");

        VuforiaTrackable gearsTarget = images.get(0);
        gearsTarget.setName("Gear");

        VuforiaTrackable toolsTarget = images.get(1);
        toolsTarget.setName("Tools");

        //add all trackables in a list for convenience
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(images);

        float botWidth = (float) 457.2;
        float feildWidth = 3580;

        /*to locate the robot based on the location of the trackable
        we must tell vuforia where the trackables are on the feild
        we do this by using transformations from the origin to tell vuforia where the objects are*/

        //place the gears on the wall
        OpenGLMatrix gearsTargetLocation = OpenGLMatrix
                .translation(-feildWidth/2, -305, 6)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gearsTarget.setLocation(gearsTargetLocation);
        RobotLog.ii(TAG, "Gear Target=%s", format(gearsTargetLocation));

        //place the tools on the wall
        OpenGLMatrix toolsTargetLocation = OpenGLMatrix
                .translation(-feildWidth/2, -762, 6)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYX,
                        AngleUnit.DEGREES, 90, 90, 0));
        toolsTarget.setLocation(toolsTargetLocation);
        RobotLog.ii(TAG, "Tool Target=%s", format(toolsTargetLocation));

        //make a another transformation matrix to describe where phone in on robot
        OpenGLMatrix phoneLocationObBot = OpenGLMatrix
                //TODO measure these values for testing and reenter
                .translation(botWidth/2, 0 , 9)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationObBot));

        //add trackable listeners
        ((VuforiaTrackableDefaultListener)gearsTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)toolsTarget.getListener()).setPhoneInformation(phoneLocationObBot, parameters.cameraDirection);

        telemetry.addData("Vuforia", " Setup is Complete");

        waitForStart();

        images.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if  (robotLocationTransform != null)
                    lastPostition = robotLocationTransform;
            }
            if (lastPostition != null){
                telemetry.addData("Position", format(lastPostition));
            } else {
                telemetry.addData("Position", "Unknown");
            }
            telemetry.update();
            idle();
        }

    }

    //small method to extract position information from a transformation matrix
    String format(OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }
}
