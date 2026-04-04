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
            0, -11.5, 0, 0); // need to make pitch smaller because -pitch = cam facing up
    public AprilTagDetection specialDetection = null;
    public int numDetected = 0;
    public VisionPortal visionPortal;
    public AprilTagDetection goalTag = null;
    Telemetry telemetry;

    public AprilTagLocalization(String team, Telemetry telemetry) {
        this.team = team;
        this.telemetry = telemetry;
    }

    public void initProcessor(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAM_POS, CAM_ORIENTATION)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(632.1683867365514, 631.3179086745744, 321.9055255193156, 232.32756377545996) // constants chatgpt gave me for cam calibration
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(CameraName.class, "webcam"));

        builder.addProcessor(aprilTag);
        builder.setCameraResolution(new Size(640, 480));
        visionPortal = builder.build();
    }

    public void applySettings() {
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

        gainControl.setGain(120);

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
        if (goalTag != null) {
            bearing = goalTag.ftcPose.bearing;
        }
        telemetry.addData("bearing", bearing);
        telemetry.addData("goalTag", goalTag);
    }

    public int getGoalID() {
        return specialDetection.id;
    }

    public AprilTagDetection getGoalTag(ArrayList<AprilTagDetection> detectedTags) {
        int teamid;
        if (team.equals("BLUE")) teamid = 20;
        else teamid = 24;

        for (AprilTagDetection tag : detectedTags) {
            if (tag.id == teamid) {
                return tag;
            }
        }

        return null;
    }

    public boolean isGoalTag(AprilTagDetection tag) {
        return tag.id == 24 || tag.id == 20;
    }
    public double filteredYaw = 0;
    public double filteredYawPrev = 0;
    public double getCurrDeg(AprilTagDetection tag) { // in pinpoint cords
        double yaw = tag.ftcPose.yaw;
        filteredYaw = utils.filter(0.03,yaw,filteredYawPrev);
//        filteredYaw = utils.updateAverage(yaw, aprilTagsTest.loopsPerUpdate);
        filteredYawPrev = filteredYaw;
        Pose2D goal;
        if (team.equals("BLUE")) {
            goal = utils.GOAL_BLUE;
        } else {
            goal = utils.GOAL_RED;
        }
        double heading = goal.getHeading(AngleUnit.DEGREES) - filteredYaw;
        if (heading < -180) {
            return heading + 360;
        } else if (heading > 180) {
            return heading - 360;
        }
        return heading;
    }

    public Pair<Double, Double> getXYToTag(AprilTagDetection tag) {
        double heading = getCurrDeg(tag);
        double x = tag.ftcPose.x;
        double y = tag.ftcPose.y + 12;
        Pair<Double, Double> rotated = utils.rotation2D(x, y, heading+90);
        Pair<Double, Double> rotFixed = new Pair<>(rotated.first, rotated.second);
        return rotFixed;
    }

    public Pair<Double, Double> getRelocXY(AprilTagDetection tag) { // in pinpoint cords
        Pose2D tagPose; // in absolute ftc coords
        if (team.equals("BLUE")) {
            tagPose = utils.GOAL_TAG_BLUE;
        } else {
            tagPose = utils.GOAL_TAG_RED;
        }
        Pair<Double, Double> xy = getXYToTag(tag); // we know what it is
        return new Pair<>(tagPose.getX(DistanceUnit.CM) + xy.first, tagPose.getX(DistanceUnit.CM) + xy.second);
    }
    public Pose2D getRobotPose(AprilTagDetection tag){//in pinpoint cords
        Pair<Double, Double> robotXY= getRelocXY(tag);
        return new Pose2D(DistanceUnit.CM, robotXY.first, robotXY.second, AngleUnit.DEGREES, getCurrDeg(tag));
    }
    Pair<Double, Double> robotPoseAbs = null;
   double robotHeadingAbs = 0;
    public void setCameraTelemetry(Telemetry telemetry) {
        if(goalTag != null){
            robotPoseAbs = getRelocXY(goalTag);
            robotHeadingAbs = getCurrDeg(goalTag);
        }
        telemetry.addData("absolute robot pose cam", robotPoseAbs);
        telemetry.addData("absolute robot heading cam", robotHeadingAbs);
    }
}