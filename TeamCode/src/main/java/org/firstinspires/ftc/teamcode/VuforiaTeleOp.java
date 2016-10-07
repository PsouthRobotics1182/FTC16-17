package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import java.util.Arrays;
import java.util.List;

@TeleOp
class VuforiaTeleOp extends OpMode {


    final String TAG = "Vuforia";
    final String LICENSE_KEY = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";
    //stores Vuforia instance
    VuforiaLocalizer vuforia;
    private OpenGLMatrix lastPostition;

    List<VuforiaTrackable> allTrackables;

    VuforiaTrackables parts;

    DcMotor leftMotor;
    DcMotor rightMotor;

    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftM");
        rightMotor = hardwareMap.dcMotor.get("rightM");

        //leftMotor = null;
        //rightMotor = null;

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

        //unit is mm
        float botWidth = (float) 457.2;
        float feildWidth = 3658;

        /*to locate the robot based on the location of the trackable
        we must tell vuforia where the trackables are on the feild
        we do this by using transformations from the origin to tell vuforia where the objects are*/

        //place the wheels on the wall.
        OpenGLMatrix wheelsTargetLocation = OpenGLMatrix
                .translation(305, feildWidth / 2, 12)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        wheelsTarget.setLocation(wheelsTargetLocation);
        RobotLog.ii(TAG, "Wheel Target=%s", format(wheelsTargetLocation));


        //place the tools on the wall
        OpenGLMatrix toolsTargetLocation = OpenGLMatrix
                .translation(-feildWidth / 2, -762, 12)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYX,
                        AngleUnit.DEGREES, 90, 90, 0));
        toolsTarget.setLocation(toolsTargetLocation);
        RobotLog.ii(TAG, "Tool Target=%s", format(toolsTargetLocation));

        //place the legos on the wall
        OpenGLMatrix legosTargetLocation = OpenGLMatrix
                .translation(-762, feildWidth / 2, 12)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        legosTarget.setLocation(legosTargetLocation);
        RobotLog.ii(TAG, "Lego Target=%s", format(legosTargetLocation));


        //place the gears on the wall
        OpenGLMatrix gearsTargetLocation = OpenGLMatrix
                .translation(-feildWidth / 2, -305, 12)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gearsTarget.setLocation(gearsTargetLocation);
        RobotLog.ii(TAG, "Gear Target=%s", format(gearsTargetLocation));


        //make a another transformation matrix to describe where phone in on robot
        //this puts it portait on the right middle of the robot, 9cm above ground
        OpenGLMatrix phoneLocationObBot = OpenGLMatrix
                //TODO measure these values for testing and reenter
                .translation(0, botWidth, 90)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 0, -90));
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
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        for (VuforiaTrackable trackable : allTrackables) {
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");
            //prints out position of trackable... hopefully
            //TODO might break it so if there is error check here

            try {
                //Puts thelocation of the trackable into a openGLmatrix
                OpenGLMatrix trackablePose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
                //matrix layout http://www.codinglabs.net/article_world_view_projection_matrix.aspx
                //{1,5,9,13}
                //{2,6,10,14}
                //{3,7,11,15}
                //{4,8,12,16}

                //matrix layout http://www.codinglabs.net/article_world_view_projection_matrix.aspx

                String readableLocation = format(trackablePose);
                String[] locationList = readableLocation.split("\\{");
                String[] cords = locationList[2].split(" ");
                telemetry.addData(trackable.getName() + " Matrix", format(trackablePose));
                telemetry.addData(trackable.getName() + " Location X", cords[0]);
                telemetry.addData(trackable.getName() + " Location Y", cords[1]);
                telemetry.addData(trackable.getName() + " Location Z", cords[2]);

            } catch (Exception e){
                telemetry.addData(trackable.getName() + " Location", " Unknown");
            }
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null)
                lastPostition = robotLocationTransform;
        }
        if (lastPostition != null) {
            String lastLocation = format(lastPostition);
            telemetry.addData("Position", lastLocation);
        } else {
            telemetry.addData("Position", "Unknown");
        }
        telemetry.update();
    }
    //small method to extract position information from a transformation matrix
    private String format(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }
}