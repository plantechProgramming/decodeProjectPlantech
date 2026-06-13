package org.firstinspires.ftc.teamcode.Misc.Utils;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Misc.RobotPose;

public class PoseFunctions {

    RobotPose robotPose;

    public PoseFunctions(RobotPose robotPose){
        this.robotPose = robotPose;
    }

    public static final double LEN_FIELD = 360.172; // in cm
    public static final double GOAL_HEADING_BLUE = -126;
    public static final double GOAL_HEADING_RED = 126;

    public static Pose2D getGoal(){ // in official ftc cords
        final double Y_GOAL_OFFSET = 16;
        final double X_GOAL_OFFSET = 5;
        if(Alliance.get() == Alliance.RED){
            return new Pose2D(DistanceUnit.CM,-LEN_FIELD/2+X_GOAL_OFFSET, LEN_FIELD/2-Y_GOAL_OFFSET,AngleUnit.DEGREES,GOAL_HEADING_RED);
        }
        else{
            return new Pose2D(DistanceUnit.CM,-LEN_FIELD/2+X_GOAL_OFFSET, -LEN_FIELD/2+Y_GOAL_OFFSET,AngleUnit.DEGREES,GOAL_HEADING_BLUE);
        }
    }

    public static Pose2D getTag(){ // in official ftc cords
        final double Y_TAG_OFFSET = 30;
        final double X_TAG_OFFSET = 35;
        if(Alliance.get() == Alliance.RED){
            return new Pose2D(DistanceUnit.CM,-LEN_FIELD/2+ X_TAG_OFFSET, LEN_FIELD/2- Y_TAG_OFFSET,AngleUnit.DEGREES,GOAL_HEADING_RED);
        }
        else{
            return new Pose2D(DistanceUnit.CM,-LEN_FIELD/2+ X_TAG_OFFSET, -LEN_FIELD/2+ Y_TAG_OFFSET,AngleUnit.DEGREES,GOAL_HEADING_BLUE);
        }
    }

    public boolean isFar(){
        return robotPose.getX() > 60;
    }
    public Pair<Double, Double> getXYDiffToPoint(Pose2D point){
        double pointX = point.getX(DistanceUnit.CM);
        double pointY = point.getY(DistanceUnit.CM);
        return new Pair<>(pointX-robotPose.getX(), pointY-robotPose.getY());
    }
    public double getPointToGoalAngle(Pose2D point1){
        double x1 = point1.getX(DistanceUnit.CM);
        double y1 = point1.getY(DistanceUnit.CM);
        double goalX = getGoal().getX(DistanceUnit.CM);
        double goalY = getGoal().getY(DistanceUnit.CM);
        return Math.toDegrees(Math.atan2(goalY-y1,goalX-x1));
    }
    public Pair<Double,Double> getXYdistToGoal(){
        return getXYDiffToPoint(getGoal());
    }
    public double getDistFromPoint(Pose2D point){
        return Math.hypot(getXYDiffToPoint(point).first, getXYDiffToPoint(point).second);
    }
    public double getDistFromGoal(){
        return Math.hypot(getXYdistToGoal().first,getXYdistToGoal().second);
    }

    public double getAngleFromPoint(Pose2D point){
        return Math.toDegrees(Math.atan2(getXYDiffToPoint(point).second, getXYDiffToPoint(point).first));
    }
    public double getAngleFromGoal(){
        double deg = Math.atan2(getXYdistToGoal().second, getXYdistToGoal().first);
        return Math.toDegrees(deg);
    }
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

    public static Pair<Double, Double> rotation2D(double x, double y, double deg){
        double rad = Math.toRadians(deg);
        return new Pair<>(x*Math.cos(rad)-y*Math.sin(rad), x*Math.sin(rad)+y*Math.cos(rad));
    }
}

