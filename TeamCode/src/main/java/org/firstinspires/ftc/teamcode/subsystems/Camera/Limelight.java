package org.firstinspires.ftc.teamcode.subsystems.Camera;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import android.util.Pair;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

    public List<LLResultTypes.ColorResult>  getLatestColorDetection() throws NullPointerException{ // LLCords
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getColorResults();
        }
        throw new NullPointerException("No valid color detection found");
    }

    public Pose2D getFilteredBotPose() throws NullPointerException{
        getLatestBotpose();
        return poseLowPass.get();
    }

    public List<Pose2D> getColorDetectionAsPose2D() throws NullPointerException{ // relative to robot
        double llHeight = 39; // cm
        double artifactHeight = 8; // approximately in cm
        List<Pose2D> XYlist = new ArrayList<>();
        List<LLResultTypes.ColorResult> colorDetections = getLatestColorDetection();
        for(LLResultTypes.ColorResult colorDetection : colorDetections){
            double xRad = Math.toRadians(colorDetection.getTargetXDegrees());
            double yRad = Math.toRadians(colorDetection.getTargetYDegrees());
            double heightDiff = llHeight - artifactHeight;

            double groundDis = heightDiff / Math.tan(yRad); // from cam to target
            double actualY = Math.cos(xRad) * groundDis;
            double actualX = Math.sin(xRad) * groundDis;
            XYlist.add(new Pose2D(DistanceUnit.CM,actualX,actualY,AngleUnit.DEGREES, Math.toDegrees(xRad)));
        }
        return XYlist;
    }

    public List<Pose2D> getRotatedColorDetection(double robotAngle){
        List<Pose2D> detectionPoses = getColorDetectionAsPose2D();
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

    public List<Pose2D> getAbsoluteColorDetection(Pose2D robotPose){ // ftc coords
        List<Pose2D> rotatedDetections = getRotatedColorDetection(robotPose.getHeading(AngleUnit.DEGREES));
        List<Pose2D> absoluteDetections = new ArrayList<>();
        for(Pose2D pose : rotatedDetections){
            double absPoseX = -pose.getY(DistanceUnit.CM) + robotPose.getX(DistanceUnit.CM);
            double absPoseY = pose.getX(DistanceUnit.CM) + robotPose.getY(DistanceUnit.CM);
            absoluteDetections.add(new Pose2D(DistanceUnit.CM, absPoseX, absPoseY, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES)));
        }
        return absoluteDetections;
    }

    public Pose2D getBestBlob(Pose2D rPose){
        List<Pose2D> poseOfDetections = getAbsoluteColorDetection(rPose);
        // TODO: make fancier
        return poseOfDetections.get(0);
    }
}
