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

import java.util.ArrayList;
import java.util.List;

public class AprilTagLocalization {
    private String team;
    private int id = 0;
    private AprilTagProcessor aprilTag;
    public String Order = "NNN";

    public double goalR_X = -1.55, goalR_Y = 1.55, goalR_Z = 95.0;
    public double goalB_X = -1.55, goalB_y = -1.55;
    public double robotToTag = 0;
    public final Position CAM_POS = new Position(DistanceUnit.CM,
            0, 0, 0, 0);

    public final Position RED_GOAL_POS = new Position(DistanceUnit.CM,
            goalR_X, goalR_Y, goalR_Z, 0);
    private final YawPitchRollAngles CAM_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0);
    public AprilTagDetection specialDetection = null;
    public int numDetected = 0;
    private VisionPortal visionPortal;

    public AprilTagLocalization(String team) {
        this.team = team;
    }

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
        List<AprilTagDetection> specialDetections = new ArrayList<AprilTagDetection>();

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
                specialDetections.add(Detection);
            }
        }
        // done to know what to remove from list
        if(team == "RED"){// inverted on perpse
            id = 20;
        }
        else if(team == "BLUE"){ // inverted on perpse
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

        /*
         * the x, y, z vals are literally the location in the field. y is toward programming table,
         * x is away from obelisk, z is just up. REQUIRES TAG LIBRARY TO BE THE RIGHT ONE TO USE,
         * OTHERWISE JUST NULL
         */


        if (specialDetection != null) {
            robotToTag = distanceToGoal(specialDetection.robotPose, specialDetection.id);
        }

    }
    public int getGoalID(){
        return specialDetection.id;
    }
    public double distanceToGoal(Pose3D robotPose, int id){
        double x = robotPose.getPosition().x, y = robotPose.getPosition().y;

        if (id == 20){
            double distanceSquared = (goalB_X - x)*(goalB_X - x) + (goalB_y - y)*(goalB_y - y);
            return Math.sqrt(distanceSquared);
        } else if (id == 24) {
            double distanceSquared = (goalR_X - x)*(goalR_X - x) + (goalR_Y - y)*(goalR_Y - y);
            return Math.sqrt(distanceSquared);
        }
        return -1;
    }
}
