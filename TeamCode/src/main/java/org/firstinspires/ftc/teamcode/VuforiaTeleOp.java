package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp
class VuforiaTeleOp extends OpMode {


    private final String TAG = "Vuforia";
    private final String LICENSE_KEY = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";
    //stores Vuforia instance
    private VuforiaLocalizer vuforia;
    private OpenGLMatrix lastPostition;

    List<VuforiaTrackable> allTrackables;

    VuforiaTrackables parts;

    public void init() {

        //setup vuforia parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //add trackable objects to vuforia instance from assets folder
        parts = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable wheelsTarget = parts.get(0);
        wheelsTarget.setName("Wheels");

        VuforiaTrackable toolsTarget = parts.get(1);
        toolsTarget.setName("Tools");

        VuforiaTrackable legosTarget = parts.get(2);
        legosTarget.setName("Legos");

        VuforiaTrackable gearsTarget = parts.get(3);
        gearsTarget.setName("Gears");

        //add all trackables in a list for convenience
        allTrackables = new ArrayList<>();
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
    }
    public void start() {

        //begin tracking the images
        parts.activate();
    }

    public void loop(){

        for (VuforiaTrackable trackable : allTrackables) {
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");
            //prints out position of trackable... hopefully
            //TODO might break it so if there is error check here

            /*try {
                OpenGLMatrix trackablePose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
                telemetry.addData(trackable.getName() + " Location", trackablePose);
            } catch (Exception e){
                telemetry.addData(trackable.getName() + " Location", "");
            }*/
            try {
                //Puts thelocation of the trackable into a openGLmatrix
                OpenGLMatrix trackablePose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
                //matrix layout http://www.codinglabs.net/article_world_view_projection_matrix.aspx

                //seperate out matrix into a array
                //seperated based on the trig equations to solve for Z axis angle
                float cosZ = trackablePose.get(0,0);
                float sinZ = trackablePose.get(0,1);
                float arcsinZ = trackablePose.get(1,0);
                float arccosZ = trackablePose.get(1,1);

                //solving the trig values
                //they should all output the Z axis rotation, therefore all be equal
                double zCos = Math.cos(cosZ);
                double zSin = Math.sin(sinZ);
                double zArcsin = Math.asin(arcsinZ);
                double zArccos = Math.acos(arccosZ);


                //matrix layout http://www.codinglabs.net/article_world_view_projection_matrix.aspx

                //outputing all calculated values to check for continiuity therefore verifying math
                //TODO verify and consolidate to average and output a single numebr
                telemetry.addData(trackable.getName() + " Rotation Z", zCos);
                telemetry.addData(trackable.getName() + " Rotation Z", zSin);
                telemetry.addData(trackable.getName() + " Rotation Z", zArcsin);
                telemetry.addData(trackable.getName() + " Rotation Z", zArccos);

            } catch (Exception e){
                telemetry.addData(trackable.getName() + " Location", " Unknown");
            }
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null)
                lastPostition = robotLocationTransform;
        }
        if (lastPostition != null) {
            //telemetry.addData("Position", format(lastPostition));
        } else {
            //telemetry.addData("Position", "Unknown");
        }
        telemetry.update();
    }
    //small method to extract position information from a transformation matrix
    private String format(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }
}
