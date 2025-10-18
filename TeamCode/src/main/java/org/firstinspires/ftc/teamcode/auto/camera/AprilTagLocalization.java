package org.firstinspires.ftc.teamcode.auto.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagLocalization {
    private AprilTagProcessor aprilTag;
    public String Order = "NNN";

    public double goalR_X= 130.0, goalR_Y= 135.0, goalR_Z = 95.0;
    public double robotToTag = 0;
    public final Position CAM_POS = new Position(DistanceUnit.CM,
            0, 0, 0, 0);

    public final Position RED_GOAL_POS = new Position(DistanceUnit.CM,
            goalR_X, goalR_Y, goalR_Z, 0);
    private final YawPitchRollAngles CAM_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0);
    public AprilTagDetection specialDetection = null;
    public int numDetected = 0;
    private VisionPortal visionPortal;

    public void initProcessor(HardwareMap hardwareMap){
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAM_POS, CAM_ORIENTATION)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(CameraName.class,"webcam"));
    }

    public void detectTags(AprilTagProcessor aprilTag) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        numDetected = currentDetections.size();
        AprilTagDetection detection = null;

        // find only non special
        for (AprilTagDetection Detection : currentDetections) {
            if (Detection.id != 20 && Detection.id != 24) {
                detection = Detection;
            }
        }
        if (detection == null) {
            Order = "NNN";
        }
        else if (detection.id == 21) {
            Order = "GPP";
        }
        else if (detection.id == 22) {
            Order = "PGP";
        }
        else if (detection.id == 23) {
            Order = "PPG";
        }
        // find only special, maybe combine with prev?
        for (AprilTagDetection Detection : currentDetections) {
            if (Detection.id != 21 && Detection.id != 22 && Detection.id != 23) {
                specialDetection = Detection;
            }
        }

        /*
         * the x, y, z vals are literally the location in the field. y is toward programming table,
         * x is away from obelisk, z is just up. REQUIRES TAG LIBRARY TO BE THE RIGHT ONE TO USE,
         * OTHERWISE JUST NULL
         */


        if (specialDetection != null) {
            robotToTag = distanceToGoal(specialDetection.robotPose);
        }

    }
    public int getGoalID(){
        return specialDetection.id;
    }
    public double distanceToGoal(Pose3D robotPose){
        //TODO: make func depend on apriltag id
        double x_pos = robotPose.getPosition().x, y_pos = robotPose.getPosition().y;
        return Math.sqrt((x_pos-goalR_X)*(x_pos-goalR_X) + (y_pos-goalR_Y)*(y_pos-goalR_Y));
    }

}
