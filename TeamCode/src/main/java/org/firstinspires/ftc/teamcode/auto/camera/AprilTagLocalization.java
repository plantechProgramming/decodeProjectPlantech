package org.firstinspires.ftc.teamcode.auto.camera;


import android.util.Pair;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teleOp.Utils;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagLocalization {
    Utils utils = new Utils();
    private String team;
    private int id = 0;
    public AprilTagProcessor aprilTag;
    public String Order = "NNN";

    double bearing = 0;
    public double robotToTag = 0;
    public final Position CAM_POS = new Position(DistanceUnit.CM,
            0, 12, 38, 0);// need to make y bigger because y = forward of robot and z bigger because the cam is higher
    private final YawPitchRollAngles CAM_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            0,-11.5,0,0); // need to make pitch smaller because -pitch = cam facing up
    public AprilTagDetection specialDetection = null;
    public int numDetected = 0;
    public VisionPortal visionPortal;
    public AprilTagDetection goalTag = null;
    Telemetry telemetry;

    public AprilTagLocalization(String team, Telemetry telemetry) {
        this.team = team;
        this.telemetry = telemetry;
    }

    public void initProcessor(HardwareMap hardwareMap){
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAM_POS, CAM_ORIENTATION)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502) // constants chatgpt gave me for cam calibration
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(CameraName.class,"webcam"));

        builder.addProcessor(aprilTag);
        builder.setCameraResolution(new Size(640, 480));
        visionPortal = builder.build();
    }
    public void applySettings(){
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        FocusControl focusControl = visionPortal.getCameraControl(FocusControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(8, TimeUnit.MILLISECONDS); // 1/120 sec

        focusControl.setMode(FocusControl.Mode.Fixed);
        focusControl.setFocusLength(25); // 10% focus

        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(3250); // 3250K

        gainControl.setGain(120); // ISO 100

        // contrast, saturation and brightness are saved in the cam?
    }

    public void detectTags() {

        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
        numDetected = currentDetections.size();
        telemetry.addData("num detected", numDetected);
//        AprilTagDetection detection = null;
//        List<AprilTagDetection> specialDetections = new ArrayList<AprilTagDetection>();
//
//        // find only non special
//        for (AprilTagDetection Detection : currentDetections) {
//            if (Detection.id != 20 && Detection.id != 24) {
//                detection = Detection;
//            }
//        }
//        if (detection == null) {
//            Order = "NNN";
//        }
//        else if (detection.id == 21) {
//            Order = "GPP";
//        }
//        else if (detection.id == 22) {
//            Order = "PGP";
//        }
//        else if (detection.id == 23) {
//            Order = "PPG";
//        }
//        // find only special, maybe combine with prev?
//        for (AprilTagDetection Detection : currentDetections) {
//            if (Detection.id != 21 && Detection.id != 22 && Detection.id != 23) {
//                specialDetections.add(Detection);
//            }
//        }
//        // done to know what to remove from list
//        if(team == "RED"){// inverted on perpse
//            id = 20;
//        }
//        else if(team == "BLUE"){ // inverted on perpse
//            id = 24;
//        }
//        if(specialDetections.size() == 2){
//            for (AprilTagDetection Detection : currentDetections){
//                if(Detection.id == id){
//                    specialDetections.remove(Detection);
//                    specialDetection = specialDetections.get(0);
//                }
//            }
//        }
//
//        /*
//         * the x, y, z vals are literally the location in the field. y is toward programming table,
//         * x is away from obelisk, z is just up. REQUIRES TAG LIBRARY TO BE THE RIGHT ONE TO USE,
//         * OTHERWISE JUST NULL
//         */
//
//
//        if (specialDetection != null) {
//            robotToTag = distanceToGoal(specialDetection.robotPose, specialDetection.id);
//        }
        goalTag = getGoalTag(currentDetections);
        if(goalTag != null){
            bearing = goalTag.ftcPose.bearing;
        }
        telemetry.addData("bearing", bearing);
        telemetry.addData("goalTag", goalTag);
    }
//    public double getWantedHeading(AprilTagDetection goalTag){
//        double bearing = goalTag.ftcPose.bearing;
//        double currDeg = goalTag.ftcPose.yaw;
//        return currDeg + bearing; // might be minus instead of thew plus
//    }
//    public double getCurrHeading(AprilTagDetection goalTag){
//        return goalTag.ftcPose.yaw;
//    }

    public int getGoalID(){
        return specialDetection.id;
    }
//    public double distanceToGoal(Pose3D robotPose, int id){
//        double x = robotPose.getPosition().x, y = robotPose.getPosition().y;
//
//        if (id == 20){
//            double distanceSquared = (goalB_X - x)*(goalB_X - x) + (goalB_y - y)*(goalB_y - y);
//            return Math.sqrt(distanceSquared);
//        } else if (id == 24) {
//            double distanceSquared = (goalR_X - x)*(goalR_X - x) + (goalR_Y - y)*(goalR_Y - y);
//            return Math.sqrt(distanceSquared);
//        }
//        return -1;
//    }

    public AprilTagDetection getGoalTag(ArrayList<AprilTagDetection> detectedTags){
        int teamid;
        if(team.equals("BLUE")) teamid = 20;
        else teamid = 24;

        for(AprilTagDetection tag: detectedTags){
            if(tag.id == teamid){
                return tag;
            }
        }

        return null;
    }

    public boolean isGoalTag(AprilTagDetection tag){
        return tag.id == 24 || tag.id == 20;
    }
//    public Pose2D getRobotPoseFromTag(AprilTagDetection tag){ // only goal tags
//        double heading = tag.ftcPose.yaw;
//        double x = tag.ftcPose.x;
//        double y = tag.ftcPose.y;
//        if(tag.id == 24){
//            heading += utils.GOAL_TAG_RED.getHeading(AngleUnit.DEGREES);
//            x += utils.GOAL_TAG_RED.getX(DistanceUnit.CM);
//            y += utils.GOAL_TAG_RED.getY(DistanceUnit.CM);
//        }
//        else if (tag.id == 20){
//            heading += utils.GOAL_TAG_BLUE.getHeading(AngleUnit.DEGREES);
//            x += utils.GOAL_TAG_BLUE.getX(DistanceUnit.CM);
//            y += utils.GOAL_TAG_BLUE.getY(DistanceUnit.CM);
//        }
//        else {
//            return null;
//        }
//        Pose2D robotPose = new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, heading); // field pose in the same cords as pinpoint
//        return robotPose;
//    }
//    public Pair<Double, Double> XYDisToGoal(AprilTagDetection tag){//tagPose
//        double yaw = tag.ftcPose.yaw;
//        double x = tag.ftcPose.x;
//        double y = tag.ftcPose.y;
//        return utils.rotation2D(x, y, yaw);
//    }
//    public double getBearingToGoal(AprilTagDetection tag){ // the bearing to the actual goal not the tag
//
//    }
    public double getWantedDeg(AprilTagDetection tag){
        double x = tag.ftcPose.x + 20;
        double y = tag.ftcPose.y + 20;
        return Math.atan2(y, x);
    }
//    public double getCurrDeg(AprilTagDetection tag){
//    }
}
