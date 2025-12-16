package org.firstinspires.ftc.teamcode.auto.camera;

import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

/**
 * FILE IS NOT USED. here for "what if we need it for some reason in comp"
 * reasons.
 */
@Autonomous(name = "aprilTags")
public class aprilTagsTest  extends LinearOpMode {

    String team;
    private AprilTagProcessor aprilTag;
    public String Order = "NNN";

    // goal x: 159,
    public double goalR_X= -159.0, goalR_Y= 135.0, goalR_Z = 95.0;
    public double robotToTag = 0;

    private final Position CAM_POS = new Position(DistanceUnit.CM,
            0, 0, 0, 0);

    private final Position RED_GOAL_POS = new Position(DistanceUnit.CM,
            goalR_X, goalR_Y, goalR_Z, 0);
    private final YawPitchRollAngles CAM_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0);
    public AprilTagDetection specialDetection = null;

    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {

        initAprilTag();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();
            telemetryAprilTag(aprilTag);
//            sleep(20);
        }

        visionPortal.close();

    }

    public void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAM_POS, CAM_ORIENTATION)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(CameraName.class,"webcam"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }


    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag(AprilTagProcessor aprilTag) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection detection = null;
        List<AprilTagDetection> specialDetections = null;
        int id = 0;

        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // find only non special
        for (AprilTagDetection Detection : currentDetections) {
            if (Detection.id != 20 && Detection.id != 24) {
                detection = Detection;
            }
        }
        if (detection == null) {
            telemetry.addLine(String.format(("\n==== (Order: NNN)")));
            Order = "NNN";
        } else if (detection.id == 21) {
            telemetry.addLine(String.format("\n==== (Order: GPP)"));
            Order = "GPP";
        } else if (detection.id == 22) {
            telemetry.addLine(String.format(("\n==== (Order: PGP)")));
            Order = "PGP";
        } else if (detection.id == 23) {
            telemetry.addLine(String.format(("\n==== (Order: PPG)")));
            Order = "PPG";
        }
        // find only special, maybe combine with prev?
        for (AprilTagDetection Detection : currentDetections) {
            if (Detection.id != 21 && Detection.id != 22 && Detection.id != 23) {
                specialDetections.add(Detection);
            }
        }
        if(team == "RED"){
            id = 20;
        }
        else if(team == "BLUE"){
            id = 24;
        }
        if(specialDetections.size() == 2){
            for (AprilTagDetection Detection : currentDetections){
                if(Detection.id == id){
                    specialDetections.remove(Detection);
                    specialDetection = specialDetections.get(0);
                }
            }
        }
        telemetry.addData("specialDetections", specialDetections);
        telemetry.addData("specialDetection", specialDetection);

        /*
         * the x, y, z vals are literally the location in the field. y is toward programming table,
         * x is away from obelisk, z is just up. REQUIRES TAG LIBRARY TO BE THE RIGHT ONE TO USE,
         * OTHERWISE JUST NULL
         */

        if (specialDetection != null) {
            telemetry.addData(" robotpose x:", specialDetection.robotPose.getPosition().x);
            robotToTag = distanceToGoal(specialDetection.robotPose);
        }

    }

    public double distanceToGoal(Pose3D robotPose){
        double x_pos = robotPose.getPosition().x, y_pos = robotPose.getPosition().y;
        return Math.sqrt(x_pos*x_pos + y_pos*y_pos);
    }
}