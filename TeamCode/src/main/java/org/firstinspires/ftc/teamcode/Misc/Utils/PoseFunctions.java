package org.firstinspires.ftc.teamcode.Misc.Utils;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseFunctions {

    public final double LEN_FIELD = 360.172; // in cm
    double yOffsetGoal = 16;//prev = 18
    double xOffsetGoal = 5; // prev = 16
    public Pose2D GOAL_RED = getGoal(xOffsetGoal,yOffsetGoal);
    public Pose2D GOAL_BLUE = getGoal(xOffsetGoal,yOffsetGoal);
    public Pose2D getGoal(double xOffset, double yOffset){
        double GOAL_HEADING_BLUE;
        double GOAL_HEADING_RED;
        if(team.equals("RED")){
            return new Pose2D(DistanceUnit.CM,-LEN_FIELD/2+xOffset, -LEN_FIELD/2 + yOffset,AngleUnit.DEGREES,GOAL_HEADING_RED);
        }
        else{
            return new Pose2D(DistanceUnit.CM,LEN_FIELD/2-xOffset, -LEN_FIELD/2 + yOffset,AngleUnit.DEGREES,GOAL_HEADING_BLUE);
        }
    }

    public boolean isFar(){
        return odometry.getPosY(DistanceUnit.CM) > 60;
    }
    public Pair<Double, Double> getXYdistToPoint(Pose2D point){
        double robotX = odometry.getPosX(DistanceUnit.CM);
        double robotY = odometry.getPosY(DistanceUnit.CM);
        double pointX = point.getX(DistanceUnit.CM);
        double pointY = point.getY(DistanceUnit.CM);
        return new Pair<>(pointX-robotX, pointY-robotY);
    }
    public double getPointToGoalAngle(Pose2D point1, String team){
        double x1 = point1.getX(DistanceUnit.CM);
        double y1 = point1.getY(DistanceUnit.CM);
        double goalX = getTeamGoal(team).getX(DistanceUnit.CM);
        double goalY = getTeamGoal(team).getY(DistanceUnit.CM);
        return Math.toDegrees(Math.atan2(goalY-y1,goalX-x1));
    }
    public Pose2D getTeamGoal(String team){
        if(team.equals("BLUE")) return GOAL_BLUE;
        if(team.equals("RED")) return GOAL_RED;
        return null;
    }
    public Pair<Double,Double> getXYdistToGoal(String team){
        if(team.equals("BLUE")){
            return getXYdistToPoint(GOAL_BLUE);
        }
        return getXYdistToPoint(GOAL_RED);
    }
    public double getDistFromPoint(Pose2D point){
        return Math.hypot(getXYdistToPoint(point).first, getXYdistToPoint(point).second);
    }
    public double getDistFromGoal(String team){
        return Math.hypot(getXYdistToGoal(team).first,getXYdistToGoal(team).second);
    }

    public double getAngleFromPoint(Pose2D point){
        return Math.toDegrees(Math.atan2(getXYdistToPoint(point).second, getXYdistToPoint(point).first));
    }
    public double getAngleFromGoal(String team){
        double deg = Math.atan2(getXYdistToGoal(team).second, getXYdistToGoal(team).first);
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
}

