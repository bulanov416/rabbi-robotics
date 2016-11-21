package org.firstinspires.ftc.teamcode.archive.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.gannacademy.libraries.VuforiaLicense;

/**
 * Created by ellagiesser on 11/6/16.
 */

@Autonomous(name="Vufonia Proof of Concept")
// @Disabled
public class VufoniaPoC extends LinearOpMode {

    public VufoniaPoC() {}

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params =
                new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = VuforiaLicense.licence;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);


        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate(); // tell vuforia to start tracking the objects
        while (opModeIsActive()) /* begin the OpMode */ {
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose =
                        ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beac.getName(), "Translation: " + translation);
                    // this next bit assumes the phone is vertical
                    double degreesToTurn =
                            Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    // use this one if it's horizontal
                    /* double deggreesToTurn =
                            Math.toDegrees(Math.atan2(translation.get(0), translation.get(2))); */
                    telemetry.addData(beac.getName(), "Degrees: " + degreesToTurn);
                }
            }
            telemetry.update();
        }

    }


}
