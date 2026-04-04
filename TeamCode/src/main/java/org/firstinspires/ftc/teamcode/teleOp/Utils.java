package org.firstinspires.ftc.teamcode.teleOp;

import static java.util.Collections.sort;

import android.util.Pair;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Utils {

    GoBildaPinpointDriver odometry;
    Telemetry telemetry;
    public Utils(Telemetry telemetry, GoBildaPinpointDriver odometry){
        this.telemetry = telemetry;
        this.odometry = odometry;
    }
    public Utils(){}
    public final double LEN_FIELD = 360.172; // in cm
    double yOffsetGoal = 20;//prev = 18
    double xOffsetGoal = 20; // prev = 16
    double yOffsetGoalTag = 30;
    double xOffsetGoalTag = 35;
    public final Pose2D GOAL_RED = new Pose2D(DistanceUnit.CM,-LEN_FIELD/2+xOffsetGoal, -LEN_FIELD/2 + yOffsetGoal,AngleUnit.DEGREES,-144);
    public final Pose2D GOAL_BLUE = new Pose2D(DistanceUnit.CM,LEN_FIELD/2-xOffsetGoal, -LEN_FIELD/2 + yOffsetGoal,AngleUnit.DEGREES,-36);
    public final Pose2D GOAL_TAG_RED = new Pose2D(DistanceUnit.CM, -LEN_FIELD/2+xOffsetGoalTag, -LEN_FIELD/2+yOffsetGoalTag, AngleUnit.DEGREES, -144);
    public final Pose2D GOAL_TAG_BLUE = new Pose2D(DistanceUnit.CM, LEN_FIELD/2-xOffsetGoalTag, -LEN_FIELD/2+yOffsetGoalTag, AngleUnit.DEGREES, -36);
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
    public Pose2D PedroPoseConverter(Pose pose){
        double x = pose.getX();
        double y = pose.getY();
        double hed = Math.toDegrees(pose.getHeading());
        double lenField = 365.76; // 144 inch to cm
        double newx = ((-lenField/144)*x)+lenField/2;
        double newy = ((-lenField/144)*y)+lenField/2;
        hed = hed - 180;
        if(hed <= 180){
            hed += 360;
        }
        return new Pose2D(DistanceUnit.CM, newx, newy, AngleUnit.DEGREES, hed);
    }

    public double angleModulo(double num){
        // fuck java * TREE(3)
        // java doesnt do mod well for negatives and doubles, so:
        // add 360 to normalize to [0,360)
        // multiply by 100 to pretend its an int, then divide to get the right val
        return ((num +360)*100%(360*100))/100;
    }

    public double getDiffBetweenAngles(double wanted, double current){// positive = left, negative = right// gyro coordinate system
        double currentError = wanted - current;
        return convertToWrapAroundAngle(currentError);
    }
    public double convertToWrapAroundAngle(double angle){
        if (angle < -180){
            angle += 360;
        }
        if (angle > 180){
            angle -= 360;
        }
        return angle;
    }
    public double getLongestDiffBetweenAngles(double wanted, double current){// positive = left, negative = right // gyro coordinate system
        double shortestPath = getDiffBetweenAngles(wanted, current);
        if(shortestPath < 0){
            return 360 - Math.abs(shortestPath);
        }
        else{
            return shortestPath - 360;
        }
    }
    public double convertGyroAngleTo360(double deg){// gets bigger to the left
        double newDeg = deg;
        if(deg<0){
             newDeg = 360 - Math.abs(deg);
        }
        return newDeg;
    }

    public Pair<Double, Double> rotation2D(double x, double y, double deg){
        double rad = Math.toRadians(deg);
        return new Pair<>(x*Math.cos(rad)-y*Math.sin(rad), x*Math.sin(rad)+y*Math.cos(rad));
    }
    public double filter(double alpha, double val, double prevVal){
        return alpha * val + (1 - alpha) * prevVal;
    }
    public Pose2D filterPose(double alpha, Pose2D pose, Pose2D lastPose){ // DO NOT USE FOR HEADING THERE ISN'T A WRAP AROUND
        double filteredX = filter(alpha, pose.getX(DistanceUnit.CM), lastPose.getX(DistanceUnit.CM));
        double filteredY = filter(alpha, pose.getY(DistanceUnit.CM), lastPose.getY(DistanceUnit.CM));
        double filteredHeading = filter(alpha, pose.getHeading(AngleUnit.DEGREES), lastPose.getHeading(AngleUnit.DEGREES));
        return new Pose2D(DistanceUnit.CM, filteredX, filteredY, AngleUnit.DEGREES, filteredHeading);
    }
    ArrayList<Double> xPos = new ArrayList<>();
    ArrayList<Double> yPos = new ArrayList<>();
    ArrayList<Double> headPos = new ArrayList<>();
    public Pose2D medianPose(Pose2D pose){
        double filteredX = updateMedian(xPos, pose.getX(DistanceUnit.CM));
        double filteredY = updateMedian(yPos, pose.getY(DistanceUnit.CM));
        double filteredHeading = updateAngleMedian(headPos, pose.getHeading(AngleUnit.DEGREES));
        return new Pose2D(DistanceUnit.CM, filteredX, filteredY, AngleUnit.DEGREES, filteredHeading);
    }
    public Pose2D subtractPoses(Pose2D pos1, Pose2D pos2){
        double subtractedX = pos1.getX(DistanceUnit.CM) - pos2.getX(DistanceUnit.CM);
        double subtractedY = pos1.getY(DistanceUnit.CM) - pos2.getY(DistanceUnit.CM);
        double subtractedHeading = getDiffBetweenAngles(pos1.getHeading(AngleUnit.DEGREES), pos2.getHeading(AngleUnit.DEGREES));
        return new Pose2D(DistanceUnit.CM, subtractedX, subtractedY, AngleUnit.DEGREES, subtractedHeading);
    }
    public boolean PoseThreshold(Pose2D pos1, Pose2D pos2, double xyThresh, double headingThresh){
        Pose2D subtractedPose = subtractPoses(pos1, pos2);
        boolean xInThresh = Math.abs(subtractedPose.getX(DistanceUnit.CM)) < xyThresh;
        boolean yInThresh = Math.abs(subtractedPose.getY(DistanceUnit.CM)) < xyThresh;
        boolean headingInThresh = Math.abs(subtractedPose.getHeading(AngleUnit.DEGREES)) < headingThresh;
        return xInThresh && yInThresh && headingInThresh;
    }
    public double updateMedian(ArrayList<Double> numbers, double val){
        numbers.add(val);
        return median(numbers);
    }
    public double median(ArrayList<Double> numbers){
        Collections.sort(numbers);
        int size = numbers.size();
        if(size % 2 == 0){
            return (numbers.get(size/2) + numbers.get(size/2 - 1) / 2);
        }
        return numbers.get(size/2);
    }
    ArrayList<Double> diffs = new ArrayList<>();
    double filteredDiffsPrev = 0;
    double alpha = 0.1;
    public double updateAngleMedian(ArrayList<Double> numbers,  double angle){
        numbers.add(angle);
        double filteredDiffs = 0;
        for (int i = 0; i < numbers.size(); i++){
            // not wrap around, cuz the sort needs that
            diffs.add(getDiffBetweenAngles(angle, numbers.get(i)));
            filteredDiffs = filter(alpha,diffs.get(diffs.size()-1),filteredDiffsPrev);
        }
        double filteredAngle = convertToWrapAroundAngle(angle + filteredDiffs);
        filteredDiffsPrev = filteredDiffs;
        diffs.clear();
        return filteredAngle;
    }

    public double getAVG(ArrayList<Double> numbers){
        double sum = 0;
        for(double number : numbers){
            sum += number;
        }
        return sum/numbers.size();
    }

    public double sum = 0;
    public double updateAverage(double curr, int loops){
        sum += curr/loops;
        return sum;
    }
    public void resetSum(){
        sum = 0;
    }
    public boolean threshold(double curr, double wanted, double threshold){
        return Math.abs(wanted - curr) < threshold;
    }
}
