package org.firstinspires.ftc.teamcode.Misc.Utils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseFunctions {
    public static Pose2D subtractPoses(Pose2D pos1, Pose2D pos2){
        double subtractedX = pos1.getX(DistanceUnit.CM) - pos2.getX(DistanceUnit.CM);
        double subtractedY = pos1.getY(DistanceUnit.CM) - pos2.getY(DistanceUnit.CM);
        double subtractedHeading = AngleFunctions.getDiffBetweenAngles(pos1.getHeading(AngleUnit.DEGREES), pos2.getHeading(AngleUnit.DEGREES));
        return new Pose2D(DistanceUnit.CM, subtractedX, subtractedY, AngleUnit.DEGREES, subtractedHeading);
    }
    public static boolean PoseThreshold(Pose2D pos1, Pose2D pos2, double xyThresh, double headingThresh) {
        Pose2D subtractedPose = subtractPoses(pos1, pos2);
        boolean xInThresh = Math.abs(subtractedPose.getX(DistanceUnit.CM)) < xyThresh;
        boolean yInThresh = Math.abs(subtractedPose.getY(DistanceUnit.CM)) < xyThresh;
        boolean headingInThresh = Math.abs(subtractedPose.getHeading(AngleUnit.DEGREES)) < headingThresh;
        return xInThresh && yInThresh && headingInThresh;
    }
}

