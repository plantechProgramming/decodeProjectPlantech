package org.firstinspires.ftc.teamcode.subsystems.Camera;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions.disBetweenPoses;

import android.util.Pair;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.AngleFunctions;
import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.Misc.Utils.filters.PoseLowPass;

import java.util.ArrayList;
import java.util.List;

public class Limelight {
    Limelight3A ll;
    PoseLowPass poseLowPass = new PoseLowPass(0.01, 0.01);
    public Limelight() {
        ll = InitMotors.ll;
    }

    public Pose3D getLatestBotpose() throws NullPointerException{ // LLCords
        LLResult result = ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                poseLowPass.update(PoseFunctions.pose3DToPose2D(result.getBotpose(), AngleUnit.DEGREES));
                return result.getBotpose();
            }
        }
        throw new NullPointerException("No valid apriltag found");
    }

    public List<ColorResult>  getLatestColorDetection() throws NullPointerException{ // LLCords
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getColorResults();
        }
        throw new NullPointerException("No valid color detection found");
    }

    public List<DetectorResult> getLatestDetectionResult() throws NullPointerException{ // LLCords
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getDetectorResults();
        }
        throw new NullPointerException("No valid detection result found");
    }

    public Pose2D getFilteredBotPose() throws NullPointerException{
        getLatestBotpose();
        return poseLowPass.get();
    }


    private Pose2D getPose2DFromXYAngle(double xDeg, double yDeg){ // the angle x and y to the target, x and y angle in deg
        double llForwardOffset = 12;// cm from lens to middle of robot
        double llHeight = 39; // cm
        double artifactHeight = 8; // approximately in cm
        double heightDiff = llHeight - artifactHeight;

        double xRad = Math.toRadians(xDeg);
        double yRad = Math.toRadians(yDeg);

        double groundDis = heightDiff / Math.tan(yRad); // from cam to target
        double actualY = Math.cos(xRad) * groundDis;
        double actualX = Math.sin(xRad) * groundDis;
        return new Pose2D(DistanceUnit.CM,actualX,actualY-llForwardOffset,AngleUnit.DEGREES, xDeg);
    }

    public List<Pose2D> getColorDetectionAsPose2D() throws NullPointerException{ // relative to robot
        List<Pose2D> XYlist = new ArrayList<>();
        List<ColorResult> colorDetections = getLatestColorDetection();
        for(ColorResult colorDetection : colorDetections){
            XYlist.add(getPose2DFromXYAngle(colorDetection.getTargetXDegrees(), colorDetection.getTargetYDegrees()));
        }
        return XYlist;
    }

    public List<Pose2D> getDetectionResultAsPose2D() throws NullPointerException{ // relative to robot
        List<Pose2D> XYlist = new ArrayList<>();
        List<DetectorResult> detectionResults = getLatestDetectionResult();
        for(DetectorResult detectionResult : detectionResults){
            XYlist.add(getPose2DFromXYAngle(detectionResult.getTargetXDegrees(), detectionResult.getTargetYDegrees()));
        }
        return XYlist;
    }
    private List<Pose2D> getRotatedDetection(double robotAngle, List<Pose2D> detectionPoses){
        List<Pose2D> rotatedDetections = new ArrayList<>();
        for(Pose2D detectionPose : detectionPoses){
            double detectionAngle = detectionPose.getHeading(AngleUnit.DEGREES);
            double absAngle = AngleFunctions.convertToWrapAroundAngle(detectionAngle + robotAngle);
            Pair<Double, Double> rotatedXY = PoseFunctions.rotation2D(detectionPose.getX(DistanceUnit.CM),
                    detectionPose.getY(DistanceUnit.CM), robotAngle);
            rotatedDetections.add(new Pose2D(DistanceUnit.CM, rotatedXY.first, rotatedXY.second, AngleUnit.DEGREES, absAngle));
        }
        return rotatedDetections;
    }

    public List<Pose2D> getRotatedColorDetection(double robotAngle){
        return getRotatedDetection(robotAngle, getColorDetectionAsPose2D());
    }

    public List<Pose2D> getRotatedDetectionResult(double robotAngle){
        return getRotatedDetection(robotAngle, getDetectionResultAsPose2D());
    }

    private List<Pose2D> getAbsoluteDetection(Pose2D robotPose, List<Pose2D> rotatedDetections){ // ftc coords
        List<Pose2D> absoluteDetections = new ArrayList<>();
        for(Pose2D pose : rotatedDetections){
            double absPoseX = -pose.getY(DistanceUnit.CM) + robotPose.getX(DistanceUnit.CM);
            double absPoseY = pose.getX(DistanceUnit.CM) + robotPose.getY(DistanceUnit.CM);
            absoluteDetections.add(new Pose2D(DistanceUnit.CM, absPoseX, absPoseY, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES)));
        }
        return absoluteDetections;
    }

    public List<Pose2D> getAbsoluteColorDetection(Pose2D robotPose){ // ftc coords
        return getAbsoluteDetection(robotPose, getRotatedColorDetection(robotPose.getHeading(AngleUnit.DEGREES)));
    }

    public List<Pose2D> getAbsoluteDetectionResult(Pose2D robotPose){ // ftc coords
        return getAbsoluteDetection(robotPose, getRotatedDetectionResult(robotPose.getHeading(AngleUnit.DEGREES)));
    }
    public List<List<Pose2D>> getBlobs(List<Pose2D> artifactPoses){
        final double MAX_NEIGHBOR_DISTANCE = 20; // in cm
        List<List<Pose2D>> blobs = new ArrayList<>();

        for(int i=0; i<artifactPoses.size(); i++){
            List<Pose2D> currBlob = new ArrayList<>();
            Pose2D fromArtifactPose = artifactPoses.get(i);
            currBlob.add(fromArtifactPose);
            for(Pose2D toArtifactPose : artifactPoses){
                if (PoseFunctions.areEqualPoses(toArtifactPose, fromArtifactPose)){
                    continue;
                }
                if(disBetweenPoses(fromArtifactPose, toArtifactPose) < MAX_NEIGHBOR_DISTANCE){
                    currBlob.add(toArtifactPose);
                }
            }
            blobs.add(currBlob);
        }
        return blobs;
    }

    public List<List<Pose2D>> getBiggestBlobs(List<Pose2D> artifactPoses){ // the biggest blobs of artifacts
        List<List<Pose2D>> blobs = getBlobs(artifactPoses);
        List<List<Pose2D>> biggestBlobs = new ArrayList<>();
        int artifactsInTheBiggestBlob = 1;

        for(List<Pose2D> blob : blobs){
            if (blob.size() > artifactsInTheBiggestBlob){
                artifactsInTheBiggestBlob = blob.size();
            }
        }
        long minNumOfArtifactsInABlob = Math.round((double)(artifactsInTheBiggestBlob + 1) / 2);
        for(List<Pose2D> blob : blobs){
            if(blob.size() >= minNumOfArtifactsInABlob){
                biggestBlobs.add(blob);
            }
        }
        return biggestBlobs;
    }
    public Pose2D getBestPoseInBlob(List<Pose2D> artifactPoses, Pose2D rPose){
        //TODO: make better
        Pose2D bestArtifact = artifactPoses.get(0);
        for(Pose2D artifactPose : artifactPoses){
            if(disBetweenPoses(artifactPose, rPose) > disBetweenPoses(bestArtifact, rPose)){
                bestArtifact = artifactPose;
            }
        }
        return bestArtifact;
    }

    public Pose2D getBestPoseForPickup(Pose2D rPose){ // the best end point of a path that intakes the most amount of artifacts
        List<Pose2D> artifactPoses = getAbsoluteDetectionResult(rPose);
        List<List<Pose2D>> biggestBlobs = getBiggestBlobs(artifactPoses);
        Pose2D bestTargetPose = getBestPoseInBlob(biggestBlobs.get(0), rPose);

        for(List<Pose2D> blob : biggestBlobs){
            Pose2D currTargetPose = getBestPoseInBlob(blob, rPose);
            if(disBetweenPoses(currTargetPose, rPose) < disBetweenPoses(bestTargetPose, rPose)){
                bestTargetPose = currTargetPose;
            }
        }
        return bestTargetPose;
    }
}
